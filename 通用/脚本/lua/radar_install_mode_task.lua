--[[
============================================================
  云帆瑞达 R60BMP1 安装方式云端切换任务

  云平台数据点：mode（开关量）
  - mode = 0：顶装，90°/330cm，边界 X±165/Y±250cm，启用自动范围
  - mode = 1：侧装，30°/220cm，边界 X±165/Y+500/Y-0cm，启用自动范围

  工作流程：
  1. 接收云端 sset，立即清除旧模式状态和串口残留。
  2. 下发安装角度，并查询读回验证。
  3. 角度验证成功后下发对应安装高度，并查询读回验证。
  4. 清空配置文件探测范围，再写入对应场景边界。
  5. 顶装和侧装都开启自动范围，并最后开启自动范围限制。
  6. 所有步骤回读一致后，才返回 ssetbck rst=0 并上报 mode。

  使用说明：
  - UART1 必须已配置为 115200、8N1。
  - 网络通道号默认为 1。
  - 本任务会独占 UART1 和网络通道 1 的接收数据，不要与另一个
    同样调用 UartGetRecChAndDel/PronetGetRecChAndDel 的任务并行运行。
============================================================
]]

function
	local tname="RADAR_INSTALL_MODE"
	local UART_CH=1
	local nid=1

	local MODE_TOP=0
	local MODE_SIDE=1
	local TOP_Z_ANGLE_X100=9000
	local SIDE_Z_ANGLE_X100=3000
	local TOP_HEIGHT_CM=330
	local SIDE_HEIGHT_CM=220
	local RANGE_X_POS_CM=165
	local RANGE_X_NEG_CM=165
	local TOP_RANGE_Y_POS_CM=250
	local TOP_RANGE_Y_NEG_CM=250
	local SIDE_RANGE_Y_POS_CM=500
	local SIDE_RANGE_Y_NEG_CM=0
	local VERIFY_DELAY_SEC=1
	local VERIFY_TIMEOUT_SEC=5
	local MAX_SET_ATTEMPTS=3
	local UNKNOWN_QUERY_INTERVAL_SEC=30
	local KNOWN_QUERY_INTERVAL_SEC=300

	local current_mode=-1
	local mode_verified=0
	local pending=nil
	local rx_buf=""
	local msg_seq=0
	local next_passive_query_at=0
	local needdup=0

	UartStopProRecCh(UART_CH)
	PronetStopProRecCh(nid)
	log.info(tname,"start, mode: 0=top, 1=side")

	local function now_ms()
		return tostring(os.time()).."000"
	end

	local function next_msg_id()
		msg_seq=msg_seq+1
		if msg_seq>999999 then msg_seq=1 end
		return tostring(os.time())..tostring(msg_seq)
	end

	local function bytes_to_hex(data)
		local out={}
		for i=1,string.len(data) do
			out[#out+1]=string.format("%02X",string.byte(data,i))
		end
		return table.concat(out," ")
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

	local function uint16_be(value)
		return math.floor(value/256)%256,value%256
	end

	local function int16_be(high,low)
		local value=high*256+low
		if value>=0x8000 then value=value-0x10000 end
		return value
	end

	local function mode_to_z_angle(mode)
		if mode==MODE_TOP then return TOP_Z_ANGLE_X100 end
		if mode==MODE_SIDE then return SIDE_Z_ANGLE_X100 end
		return nil
	end

	local function mode_to_height(mode)
		if mode==MODE_TOP then return TOP_HEIGHT_CM end
		if mode==MODE_SIDE then return SIDE_HEIGHT_CM end
		return nil
	end

	local function z_angle_to_mode(z_angle_x100)
		if z_angle_x100==TOP_Z_ANGLE_X100 then return MODE_TOP end
		if z_angle_x100==SIDE_Z_ANGLE_X100 then return MODE_SIDE end
		return -1
	end

	local function uart_send(data)
		local called=false
		local result=nil
		local api="none"

		-- Air780EP/LuatOS 的 uart.write 直接写物理串口，优先使用。
		-- UartSetSendCh 属于平台通道接口，在部分固件中只写平台缓冲区，
		-- 即使函数调用未报错也可能没有真正下发到雷达，因此仅作为回退。
		if type(uart)=="table" and type(uart.write)=="function" then
			api="uart.write"
			called,result=pcall(uart.write,UART_CH,data)
			if called and type(result)=="number" and result<=0 then called=false end
			if called and result==false then called=false end
		end
		if not called and type(UartSetSendCh)=="function" then
			api="UartSetSendCh"
			called,result=pcall(UartSetSendCh,UART_CH,data)
			if called and result==false then called=false end
		end

		if not called then
			log.warn(tname,"uart send failed api="..api.." result="..tostring(result))
			return false
		end
		log.info(tname,"uart tx api="..api.." result="..tostring(result).." hex="..bytes_to_hex(data))
		return true
	end

	local function send_install_mode(mode)
		local z_angle=mode_to_z_angle(mode)
		if not z_angle then return false end
		local z_high,z_low=uint16_be(z_angle)
		return uart_send(build_frame(0x06,0x01,{0x00,0x00,0x00,0x00,z_high,z_low}))
	end

	local function send_install_height(mode)
		local height=mode_to_height(mode)
		if not height then return false end
		local high,low=uint16_be(height)
		return uart_send(build_frame(0x06,0x02,{high,low}))
	end

	local function query_install_mode()
		return uart_send(build_frame(0x06,0x81,{0x0F}))
	end

	local function query_install_height()
		return uart_send(build_frame(0x06,0x82,{0x0F}))
	end

	local function mode_to_range(mode)
		if mode==MODE_TOP then
			return RANGE_X_POS_CM,RANGE_X_NEG_CM,TOP_RANGE_Y_POS_CM,TOP_RANGE_Y_NEG_CM
		end
		if mode==MODE_SIDE then
			return RANGE_X_POS_CM,RANGE_X_NEG_CM,SIDE_RANGE_Y_POS_CM,SIDE_RANGE_Y_NEG_CM
		end
		return nil
	end

	local function send_clear_config_range()
		-- 0x17 的 9 字节全零配置表示无门、无宽高、无坐标点。
		return uart_send(build_frame(0x07,0x17,{0,0,0,0,0,0,0,0,0}))
	end

	local function query_config_range()
		return uart_send(build_frame(0x07,0x97,{0x0F}))
	end

	local function send_fixed_boundary(mode)
		local xp,xn,yp,yn=mode_to_range(mode)
		if not xp then return false end
		local xp_h,xp_l=uint16_be(xp)
		local xn_h,xn_l=uint16_be(xn)
		local yp_h,yp_l=uint16_be(yp)
		local yn_h,yn_l=uint16_be(yn)
		return uart_send(build_frame(0x07,0x09,{0,xp_h,xp_l,xn_h,xn_l,yp_h,yp_l,yn_h,yn_l}))
	end

	local function query_boundary()
		return uart_send(build_frame(0x07,0x89,{0x0F}))
	end

	local function send_auto_range_use(enabled)
		return uart_send(build_frame(0x07,0x0C,{enabled and 1 or 0}))
	end

	local function send_auto_range_limit(enabled)
		return uart_send(build_frame(0x07,0x08,{enabled and 1 or 0}))
	end

	local function send_cloud(obj)
		local payload=json.encode(obj)
		log.info(tname,"cloud tx: "..payload)
		PronetSetSendCh(nid,payload)
	end

	local function send_sset_back(did,rst)
		send_cloud({
			cmd="ssetbck",
			did=tostring(did or ""),
			rst=rst,
			times=now_ms()
		})
	end

	local function send_sget_back(did,wants_mode)
		local b={
			cmd="sgetbck",
			did=tostring(did or ""),
			rst=0,
			times=now_ms(),
			param={}
		}
		if wants_mode then
			if mode_verified==1 and (current_mode==MODE_TOP or current_mode==MODE_SIDE) then
				b.param.mode=current_mode
			else
				b.rst=1
			end
		end
		send_cloud(b)
	end

	local function send_mode_dup()
		if mode_verified~=1 then return end
		if current_mode~=MODE_TOP and current_mode~=MODE_SIDE then return end
		send_cloud({
			cmd="dup",
			did=next_msg_id(),
			times=now_ms(),
			param={mode=current_mode}
		})
		needdup=0
	end

	local function send_dreg()
		send_cloud({
			cmd="dreg",
			did=next_msg_id(),
			iccid=mobile.iccid() or "",
			imsi=mobile.imsi() or "",
			imei=mobile.imei() or "",
			pver="Radar_Install_Mode_1.2",
			fver="FW_1.0.0",
			times=now_ms()
		})
	end

	local function normalize_mode(value)
		if type(value)=="boolean" then return value and 1 or 0 end
		if type(value)=="string" then value=tonumber(value) end
		if value==MODE_TOP or value==MODE_SIDE then return value end
		return nil
	end

	local function find_mode_value(value,depth)
		depth=depth or 0
		if depth>4 or type(value)~="table" then return nil end

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

		if value.datapoint~=nil then
			local mode=find_mode_value(value.datapoint,depth+1)
			if mode~=nil then return mode end
		end

		for _,item in pairs(value) do
			if type(item)=="table" then
				local mode=find_mode_value(item,depth+1)
				if mode~=nil then return mode end
			end
		end
		return nil
	end

	local function sget_wants_mode(param)
		if type(param)~="table" then return true end
		if param.mode~=nil then return true end
		local list=param.datapoint or param
		if type(list)~="table" then return true end
		if #list==0 then return true end
		for _,item in ipairs(list) do
			if item=="mode" then return true end
			if type(item)=="table" and (item.id=="mode" or item.name=="mode") then return true end
		end
		return false
	end

	local finish_mode_change
	local start_phase

	local function send_phase(phase,mode)
		if phase=="angle" then return send_install_mode(mode) end
		if phase=="height" then return send_install_height(mode) end
		if phase=="config_clear" then return send_clear_config_range() end
		if phase=="auto_limit_off" then return send_auto_range_limit(false) end
		if phase=="auto_use_off" then return send_auto_range_use(false) end
		if phase=="boundary" then return send_fixed_boundary(mode) end
		if phase=="auto_use_on" then return send_auto_range_use(true) end
		if phase=="auto_limit_on" then return send_auto_range_limit(true) end
		return false
	end

	local function query_phase(phase)
		if phase=="angle" then return query_install_mode() end
		if phase=="height" then return query_install_height() end
		if phase=="config_clear" then return query_config_range() end
		return query_boundary()
	end

	start_phase=function(phase)
		if not pending then return false end
		if not send_phase(phase,pending.mode) then
			finish_mode_change(false)
			return false
		end
		local now=os.time()
		pending.phase=phase
		pending.attempt=1
		pending.query_at=now+VERIFY_DELAY_SEC
		pending.deadline=now+VERIFY_TIMEOUT_SEC
		pending.query_sent=false
		log.info(tname,"config phase started: "..phase)
		return true
	end

	local function start_mode_change(mode,did)
		if pending then
			log.warn(tname,"mode change busy")
			send_sset_back(did,3)
			return
		end

		current_mode=-1
		mode_verified=0
		rx_buf=""
		UartGetRecChAndDel(UART_CH)

		pending={
			mode=mode,
			did=did
		}
		log.info(tname,"mode change requested: "..mode)
		start_phase("angle")
	end

	finish_mode_change=function(success)
		if not pending then return end
		local did=pending.did
		local mode=pending.mode
		pending=nil

		if success then
			current_mode=mode
			mode_verified=1
			needdup=1
			send_sset_back(did,0)
			log.info(tname,"mode verified: "..mode)
		else
			mode_verified=0
			send_sset_back(did,2)
			log.warn(tname,"mode verification failed: "..mode)
		end
	end

	local function on_install_angle(x_angle,y_angle,z_angle)
		local detected_mode=z_angle_to_mode(z_angle)
		log.info(tname,string.format(
			"install angle readback: x=%.2f y=%.2f z=%.2f mode=%d",
			x_angle/100,y_angle/100,z_angle/100,detected_mode
		))

		if pending then
			if pending.phase=="angle" and pending.query_sent and x_angle==0 and y_angle==0 and detected_mode==pending.mode then
				start_phase("height")
			end
			return
		end

		if x_angle==0 and y_angle==0 and detected_mode~=-1 then
			if current_mode~=detected_mode or mode_verified==0 then
				current_mode=detected_mode
				mode_verified=1
				needdup=1
			end
		else
			current_mode=-1
			mode_verified=0
		end
	end

	local function on_install_height(height_cm)
		log.info(tname,"install height readback: "..height_cm.."cm")
		if pending and pending.phase=="height" and pending.query_sent and height_cm==mode_to_height(pending.mode) then
			start_phase("config_clear")
		end
	end

	local function config_range_is_empty(frame,data_len)
		if data_len~=9 then return false end
		for i=7,15 do
			if string.byte(frame,i)~=0 then return false end
		end
		return true
	end

	local function boundary_matches(frame,data_len,mode)
		if data_len<9 or string.byte(frame,7)~=0 then return false end
		local xp,xn,yp,yn=mode_to_range(mode)
		local got_xp=string.byte(frame,8)*256+string.byte(frame,9)
		local got_xn=string.byte(frame,10)*256+string.byte(frame,11)
		local got_yp=string.byte(frame,12)*256+string.byte(frame,13)
		local got_yn=string.byte(frame,14)*256+string.byte(frame,15)
		return got_xp==xp and got_xn==xn and got_yp==yp and got_yn==yn
	end

	local function on_config_range(frame,data_len)
		if pending and pending.phase=="config_clear" and pending.query_sent and config_range_is_empty(frame,data_len) then
			start_phase("boundary")
		end
	end

	local function on_boundary(frame,data_len)
		if not pending or not pending.query_sent or data_len<1 then return end
		local range_mode=string.byte(frame,7)
		if pending.phase=="auto_limit_off" then
			start_phase("auto_use_off")
		elseif pending.phase=="auto_use_off" and range_mode==0 then
			start_phase("boundary")
		elseif pending.phase=="boundary" and boundary_matches(frame,data_len,pending.mode) then
			start_phase("auto_use_on")
		elseif pending.phase=="auto_use_on" and range_mode==1 then
			start_phase("auto_limit_on")
		elseif pending.phase=="auto_limit_on" and range_mode==1 then
			finish_mode_change(true)
		end
	end

	local function parse_frame(frame,data_len)
		local control_word=string.byte(frame,3)
		local command_word=string.byte(frame,4)
		if control_word==0x06 and command_word==0x81 and data_len>=6 then
			local x_angle=int16_be(string.byte(frame,7),string.byte(frame,8))
			local y_angle=int16_be(string.byte(frame,9),string.byte(frame,10))
			local z_angle=int16_be(string.byte(frame,11),string.byte(frame,12))
			on_install_angle(x_angle,y_angle,z_angle)
		elseif control_word==0x06 and command_word==0x82 and data_len>=2 then
			local height_cm=string.byte(frame,7)*256+string.byte(frame,8)
			on_install_height(height_cm)
		elseif control_word==0x07 and command_word==0x97 then
			on_config_range(frame,data_len)
		elseif control_word==0x07 and command_word==0x89 then
			on_boundary(frame,data_len)
		end
	end

	local function try_parse_frame()
		local head=string.find(rx_buf,string.char(0x53,0x59),1,true)
		if not head then
			if string.len(rx_buf)>1 then rx_buf=string.sub(rx_buf,-1) end
			return false
		end
		if head>1 then rx_buf=string.sub(rx_buf,head) end
		if string.len(rx_buf)<9 then return false end

		local data_len=string.byte(rx_buf,5)*256+string.byte(rx_buf,6)
		if data_len>1024 then
			rx_buf=string.sub(rx_buf,2)
			return true
		end

		local frame_len=9+data_len
		if string.len(rx_buf)<frame_len then return false end
		local frame=string.sub(rx_buf,1,frame_len)
		rx_buf=string.sub(rx_buf,frame_len+1)

		if string.byte(frame,frame_len-1)~=0x54 or string.byte(frame,frame_len)~=0x43 then
			log.warn(tname,"invalid frame tail")
			return true
		end

		local checksum=0
		for i=1,6+data_len do checksum=(checksum+string.byte(frame,i))%256 end
		if checksum~=string.byte(frame,7+data_len) then
			log.warn(tname,"invalid frame checksum")
			return true
		end

		log.info(tname,"uart rx: "..bytes_to_hex(frame))
		parse_frame(frame,data_len)
		return true
	end

	local function read_uart()
		local data=UartGetRecChAndDel(UART_CH)
		if data and string.len(data)>0 then
			rx_buf=rx_buf..data
			while try_parse_frame() do end
		end
	end

	local function update_pending(now)
		if not pending then return end

		if not pending.query_sent and now>=pending.query_at then
			pending.query_sent=query_phase(pending.phase)
		end

		if now<pending.deadline then return end
		if pending.attempt>=MAX_SET_ATTEMPTS then
			finish_mode_change(false)
			return
		end

		pending.attempt=pending.attempt+1
		local sent=send_phase(pending.phase,pending.mode)
		if not sent then
			finish_mode_change(false)
			return
		end
		pending.query_at=now+VERIFY_DELAY_SEC
		pending.deadline=now+VERIFY_TIMEOUT_SEC
		pending.query_sent=false
		log.warn(tname,"retry mode change, attempt="..pending.attempt)
	end

	local function handle_cloud_message(raw)
		log.info(tname,"cloud rx: "..raw)
		local ok,obj=pcall(json.decode,raw)
		if not ok or type(obj)~="table" then
			log.warn(tname,"invalid cloud json")
			return
		end

		if obj.cmd=="sset" then
			local mode=find_mode_value(obj.param)
			if mode==nil then mode=normalize_mode(obj.mode) end
			if mode==nil then
				log.warn(tname,"invalid mode, expected 0 or 1")
				send_sset_back(obj.did,1)
			else
				start_mode_change(mode,obj.did)
			end
		elseif obj.cmd=="sget" then
			send_sget_back(obj.did,sget_wants_mode(obj.param))
		elseif obj.cmd=="dupbck" then
			log.info(tname,"dupbck rst="..tostring(obj.rst))
		elseif obj.cmd=="dregbck" or obj.cmd=="regbck" then
			log.info(tname,"register back did="..tostring(obj.did))
		end
	end

	sys.wait(2000)
	send_dreg()
	query_install_mode()
	next_passive_query_at=os.time()+UNKNOWN_QUERY_INTERVAL_SEC

	while true do
		read_uart()

		local netr=PronetGetRecChAndDel(nid)
		if netr then handle_cloud_message(netr) end

		local now=os.time()
		update_pending(now)

		if not pending and now>=next_passive_query_at then
			query_install_mode()
			if current_mode==-1 then
				next_passive_query_at=now+UNKNOWN_QUERY_INTERVAL_SEC
			else
				next_passive_query_at=now+KNOWN_QUERY_INTERVAL_SEC
			end
		end

		if needdup==1 then send_mode_dup() end
		sys.wait(100)
	end
end
