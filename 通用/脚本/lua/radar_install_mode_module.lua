--[[
  R60BMP1 安装方式切换模块（可嵌入现有 DTU Lua 主任务）

  这是一个“无主循环、无网络读取、无串口读取”的纯模块：
  - 主任务继续负责 UartGetRecChAndDel、PronetGetRecChAndDel 和业务帧解析。
  - 主任务收到完整且校验通过的雷达帧后，调用 controller:handle_frame(frame)。
  - 主任务每 100ms 调用 controller:update(os.time())。
  - 云端收到 sset 后调用 controller:request(mode, did)。

  配置顺序固定为：
    安装角度 -> 安装高度 -> 清空配置文件探测范围 -> 场景边界 -> 自动范围模式

  mode:
    0 = 顶装：Z=90°，高度=330cm
    1 = 侧装：Z=30°，高度=220cm

  依赖通过 opts 注入，避免本模块抢占现有任务的 UART/网络队列：
    opts.send_uart(data)                         必填，成功返回 true/nil，失败返回 false
    opts.on_config_start(mode, profile)          可选，开始前清空业务状态和 rx_buf
    opts.on_config_end(ok, mode, rst, reason)    可选，完成/失败后恢复业务状态
    opts.now()                                   可选，默认 os.time()
    opts.max_attempts                            可选，默认 3
    opts.verify_delay_sec                       可选，默认 1
    opts.verify_timeout_sec                     可选，默认 5

  典型接入：
    local controller=InstallMode.new({
      send_uart=function(data)
        if type(uart)=="table" and type(uart.write)=="function" then
          return uart.write(1,data)
        end
        return UartSetSendCh(1,data)
      end,
      on_config_start=function(mode)
        rx_buf=""
        clear_all("install_mode_start")
        radar_ready=0
      end,
      on_config_end=function(ok,mode,rst)
        if ok then apply_scene_profile(mode) end
        needdup=1
      end
    })

    -- 完整帧校验后，先交给模块；返回 true 表示该帧是配置回读帧
    if controller:handle_frame(frame) then return true end
    -- 主循环中
    controller:update(os.time())
]]

local M={}

M.MODE_TOP=0
M.MODE_SIDE=1
M.TOP_HEIGHT_CM=330
M.SIDE_HEIGHT_CM=220
M.TOP_Z_ANGLE_X100=9000
M.SIDE_Z_ANGLE_X100=3000
M.RANGE_X_POS_CM=165
M.RANGE_X_NEG_CM=165
M.TOP_RANGE_Y_POS_CM=250
M.TOP_RANGE_Y_NEG_CM=250
M.SIDE_RANGE_Y_POS_CM=500
M.SIDE_RANGE_Y_NEG_CM=0

local function normalize_mode(value)
	if type(value)=="boolean" then return value and 1 or 0 end
	if type(value)=="string" then value=tonumber(value) end
	if value==M.MODE_TOP or value==M.MODE_SIDE then return value end
	return nil
end

-- 兼容常见的 sset 结构：{mode=0}、{datapoint={mode=0}}、
-- {id="mode",value=0}、{datapoint={{id="mode",value=0}}} 等。
function M.extract_mode(value,depth)
	depth=depth or 0
	if depth>5 or type(value)~="table" then return nil end

	if value.mode~=nil then
		local mode=normalize_mode(value.mode)
		if mode~=nil then return mode end
	end

	local id=value.id or value.name or value.key or value.identifier
	if id=="mode" then
		local mode=normalize_mode(value.value)
		if mode==nil then mode=normalize_mode(value.val) end
		if mode==nil then mode=normalize_mode(value.data) end
		if mode~=nil then return mode end
	end

	for _,item in pairs(value) do
		if type(item)=="table" then
			local mode=M.extract_mode(item,depth+1)
			if mode~=nil then return mode end
		end
	end
	return nil
end

function M.profile(mode)
	mode=normalize_mode(mode)
	if mode==nil then return nil end
	if mode==M.MODE_TOP then
		return {mode=mode,name="top",z_angle_x100=M.TOP_Z_ANGLE_X100,height_cm=M.TOP_HEIGHT_CM,
			x_pos_cm=M.RANGE_X_POS_CM,x_neg_cm=M.RANGE_X_NEG_CM,
			y_pos_cm=M.TOP_RANGE_Y_POS_CM,y_neg_cm=M.TOP_RANGE_Y_NEG_CM,auto_range=true}
	end
	return {mode=mode,name="side",z_angle_x100=M.SIDE_Z_ANGLE_X100,height_cm=M.SIDE_HEIGHT_CM,
		x_pos_cm=M.RANGE_X_POS_CM,x_neg_cm=M.RANGE_X_NEG_CM,
		y_pos_cm=M.SIDE_RANGE_Y_POS_CM,y_neg_cm=M.SIDE_RANGE_Y_NEG_CM,auto_range=true}
end

local function uint16_be(value)
	return math.floor(value/256)%256,value%256
end

local function int16_be(high,low)
	local value=high*256+low
	if value>=0x8000 then value=value-0x10000 end
	return value
end

local function build_frame(control_word,command_word,data_bytes)
	local bytes={0x53,0x59,control_word,command_word,0x00,#data_bytes}
	for i=1,#data_bytes do bytes[#bytes+1]=data_bytes[i] end
	local checksum=0
	for i=1,#bytes do checksum=(checksum+bytes[i])%256 end
	bytes[#bytes+1]=checksum
	bytes[#bytes+1]=0x54
	bytes[#bytes+1]=0x43
	local parts={}
	for i=1,#bytes do parts[i]=string.char(bytes[i]) end
	return table.concat(parts)
end

M.build_frame=build_frame

local function valid_frame(frame)
	if type(frame)~="string" or #frame<9 then return false,nil end
	if string.byte(frame,1)~=0x53 or string.byte(frame,2)~=0x59 then return false,nil end
	local data_len=string.byte(frame,5)*256+string.byte(frame,6)
	local frame_len=9+data_len
	if #frame~=frame_len then return false,nil end
	if string.byte(frame,frame_len-1)~=0x54 or string.byte(frame,frame_len)~=0x43 then return false,nil end
	local checksum=0
	for i=1,6+data_len do checksum=(checksum+string.byte(frame,i))%256 end
	if checksum~=string.byte(frame,7+data_len) then return false,nil end
	return true,data_len
end

function M.new(opts)
	opts=opts or {}
	assert(type(opts.send_uart)=="function","radar_install_mode_module: opts.send_uart is required")

	local self={
		state="idle",
		mode=-1,
		pending=nil,
		last_error=nil,
		max_attempts=opts.max_attempts or 3,
		verify_delay_sec=opts.verify_delay_sec or 1,
		verify_timeout_sec=opts.verify_timeout_sec or 5,
	}

	local function now()
		if type(opts.now)=="function" then return opts.now() end
		return os.time()
	end

	local function callback_config_end(ok,mode,rst,reason)
		if type(opts.on_config_end)=="function" then
			opts.on_config_end(ok,mode,rst,reason)
		end
	end

	local function send(data)
		local called,result=pcall(opts.send_uart,data)
		return called and result~=false
	end

	local function set_angle(profile)
		local z_high,z_low=uint16_be(profile.z_angle_x100)
		return send(build_frame(0x06,0x01,{0x00,0x00,0x00,0x00,z_high,z_low}))
	end

	local function query_angle()
		return send(build_frame(0x06,0x81,{0x0F}))
	end

	local function set_height(profile)
		local high,low=uint16_be(profile.height_cm)
		return send(build_frame(0x06,0x02,{high,low}))
	end

	local function query_height()
		return send(build_frame(0x06,0x82,{0x0F}))
	end

	local function clear_config_range()
		return send(build_frame(0x07,0x17,{0,0,0,0,0,0,0,0,0}))
	end

	local function query_config_range()
		return send(build_frame(0x07,0x97,{0x0F}))
	end

	local function set_boundary(profile)
		local xph,xpl=uint16_be(profile.x_pos_cm)
		local xnh,xnl=uint16_be(profile.x_neg_cm)
		local yph,ypl=uint16_be(profile.y_pos_cm)
		local ynh,ynl=uint16_be(profile.y_neg_cm)
		return send(build_frame(0x07,0x09,{0,xph,xpl,xnh,xnl,yph,ypl,ynh,ynl}))
	end

	local function query_boundary()
		return send(build_frame(0x07,0x89,{0x0F}))
	end

	local function set_auto_use(enabled)
		return send(build_frame(0x07,0x0C,{enabled and 1 or 0}))
	end

	local function set_auto_limit(enabled)
		return send(build_frame(0x07,0x08,{enabled and 1 or 0}))
	end

	local function finish(ok,rst,reason)
		local pending=self.pending
		if not pending then return end
		self.pending=nil
		self.state="idle"
		self.last_error=ok and nil or reason
		if ok then self.mode=pending.mode end
		callback_config_end(ok,pending.mode,rst,reason)
	end

	local function send_phase(phase,profile)
		if phase=="angle" then return set_angle(profile) end
		if phase=="height" then return set_height(profile) end
		if phase=="config_clear" then return clear_config_range() end
		if phase=="auto_limit_off" then return set_auto_limit(false) end
		if phase=="auto_use_off" then return set_auto_use(false) end
		if phase=="boundary" then return set_boundary(profile) end
		if phase=="auto_use_on" then return set_auto_use(true) end
		if phase=="auto_limit_on" then return set_auto_limit(true) end
		return false
	end

	local function query_phase(phase)
		if phase=="angle" then return query_angle() end
		if phase=="height" then return query_height() end
		if phase=="config_clear" then return query_config_range() end
		return query_boundary()
	end

	local function begin_phase(phase)
		local pending=self.pending
		if not pending then return false end
		pending.phase=phase
		pending.attempt=1
		pending.query_sent=false
		pending.query_at=now()+self.verify_delay_sec
		pending.deadline=now()+self.verify_timeout_sec

		local sent=send_phase(phase,pending.profile)
		if not sent then
			finish(false,2,phase.."_write_failed")
			return false
		end
		return true
	end

	function self:request(value,token)
		local mode=normalize_mode(value)
		if mode==nil then
			callback_config_end(false,nil,1,"invalid_mode")
			return false,1
		end
		if self.pending then
			callback_config_end(false,mode,3,"busy")
			return false,3
		end

		local profile=M.profile(mode)
		self.last_error=nil
		self.pending={mode=mode,profile=profile,token=token,phase="angle"}
		if type(opts.on_config_start)=="function" then opts.on_config_start(mode,profile) end
		if not begin_phase("angle") then return false,2 end
		return true,0
	end

	function self:handle_frame(frame)
		local valid,data_len=valid_frame(frame)
		if not valid then return false end
		local control_word=string.byte(frame,3)
		local command_word=string.byte(frame,4)

		-- 返回 true 表示该帧属于安装参数通道，主任务不应再当业务帧处理。
		if control_word==0x06 and command_word==0x81 and data_len>=6 then
			local x=int16_be(string.byte(frame,7),string.byte(frame,8))
			local y=int16_be(string.byte(frame,9),string.byte(frame,10))
			local z=int16_be(string.byte(frame,11),string.byte(frame,12))
			if self.pending and self.pending.phase=="angle" and self.pending.query_sent then
				if x==0 and y==0 and z==self.pending.profile.z_angle_x100 then
					begin_phase("height")
				end
			end
			return true
		end

		if control_word==0x06 and command_word==0x82 and data_len>=2 then
			local height=string.byte(frame,7)*256+string.byte(frame,8)
			if self.pending and self.pending.phase=="height" and self.pending.query_sent then
				if height==self.pending.profile.height_cm then begin_phase("config_clear") end
			end
			return true
		end

		if control_word==0x07 and command_word==0x97 then
			if self.pending and self.pending.phase=="config_clear" and self.pending.query_sent and data_len==9 then
				local empty=true
				for i=7,15 do if string.byte(frame,i)~=0 then empty=false break end end
				if empty then
					if self.pending.profile.auto_range then begin_phase("boundary") else begin_phase("auto_limit_off") end
				end
			end
			return true
		end

		if control_word==0x07 and command_word==0x89 and data_len>=1 then
			local pending=self.pending
			if pending and pending.query_sent then
				local range_mode=string.byte(frame,7)
				if pending.phase=="auto_limit_off" then
					begin_phase("auto_use_off")
				elseif pending.phase=="auto_use_off" and range_mode==0 then
					begin_phase("boundary")
				elseif pending.phase=="boundary" and data_len>=9 and range_mode==0 then
					local p=pending.profile
					local xp=string.byte(frame,8)*256+string.byte(frame,9)
					local xn=string.byte(frame,10)*256+string.byte(frame,11)
					local yp=string.byte(frame,12)*256+string.byte(frame,13)
					local yn=string.byte(frame,14)*256+string.byte(frame,15)
					if xp==p.x_pos_cm and xn==p.x_neg_cm and yp==p.y_pos_cm and yn==p.y_neg_cm then
						if p.auto_range then begin_phase("auto_use_on") else finish(true,0,nil) end
					end
				elseif pending.phase=="auto_use_on" and range_mode==1 then
					begin_phase("auto_limit_on")
				elseif pending.phase=="auto_limit_on" and range_mode==1 then
					finish(true,0,nil)
				end
			end
			return true
		end
		return false
	end

	function self:update(timestamp)
		if not self.pending then return end
		local current_time=timestamp or now()
		local pending=self.pending
		if not pending.query_sent and current_time>=pending.query_at then
			pending.query_sent=query_phase(pending.phase)
			if not pending.query_sent then finish(false,2,pending.phase.."_query_failed") end
			return
		end
		if current_time<pending.deadline then return end
		if pending.attempt>=self.max_attempts then
			finish(false,2,pending.phase.."_verify_timeout")
			return
		end
		pending.attempt=pending.attempt+1
		pending.query_sent=false
		pending.query_at=current_time+self.verify_delay_sec
		pending.deadline=current_time+self.verify_timeout_sec
		local sent=send_phase(pending.phase,pending.profile)
		if not sent then finish(false,2,pending.phase.."_write_failed") end
	end

	function self:is_busy() return self.pending~=nil end
	function self:get_mode() return self.mode end
	function self:get_pending_phase() return self.pending and self.pending.phase or nil end
	function self:get_profile() return self.pending and self.pending.profile or M.profile(self.mode) end

	return self
end

return M
