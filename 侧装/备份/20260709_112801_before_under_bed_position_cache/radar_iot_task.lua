--[[
============================================================
  云帆瑞达 60G 毫米波雷达 -> 银尔达 IOT 平台 DTU 任务脚本
  使用方式: 复制全部内容 -> 粘贴到 DTU 平台"任务"编辑区
  通道号: 1 (与网络通道编号一致)
============================================================
]]

function
	local tname="RADAR_IOT"
	log.info(tname,"=== Radar IOT Task Start ===")
	
	-- 网络通道号 (与 DTU 9.6 配置的网络通道编号一致)
	local nid=1
	
	-- IOT系统保留字段
	local pver,fver="Radar_DTU_1.0","FW_1.0.0"

	-- 寝室场景姿态判断参数，单位cm
	-- 床区坐标需要按现场实测traj_x/traj_y调整；默认先覆盖常见前方检测区域。
	local BED_HEIGHT_CM=212
	local BED_ON_TOLERANCE_CM=25
	local BED_SIT_HEIGHT_CM=220
	local TARGET_HEIGHT_MIN_CM=20
	local TARGET_HEIGHT_MAX_CM=245
	local SIDE_AREA_X_MIN_ABS=35
	local BED_AREA_X_MIN_ABS=60
	local SIDE_AREA_X_MAX_ABS=300
	local BED_Y_MIN=0
	local BED_Y_MAX=500
	local BED_SURFACE_Y_MIN=300
	local STAND_HEIGHT_CM=130
	local SIT_HEIGHT_CM=70
	local POSTURE_HYSTERESIS_CM=10
	local POSTURE_CONFIRM_FRAMES=2
	local PEOPLE_COUNT_MAX=10
	local PEOPLE_CHANGE_CONFIRM_FRAMES=3
	local PEOPLE_ZERO_CONFIRM_FRAMES=5
	local ACCURATE_CONFLICT_CONFIRM_FRAMES=3
	local TARGET_COUNT_CONFIRM_FRAMES=2
	local TARGET_COUNT_ZERO_CONFIRM_FRAMES=3
	local TARGET_CLEAR_CONFIRM_FRAMES=3
	local UNDER_BED_PEOPLE_COUNT_MAX=7
	local UNDER_BED_COUNT_CONFIRM_FRAMES=2
	local UNDER_BED_COUNT_ZERO_CONFIRM_FRAMES=5
	
	-- 雷达业务数据缓存变量
	local human_present=0   -- 人体存在: 0=无人, 1=有人
	local motion=0          -- 运动状态: 0=无运动, 1=静止, 2=活跃
	local body_move=0       -- 体动参数: 0-100
	local people_min=0      -- 标准人数最小值，平时跟随实时滤波，精准人数用于校准
	local people_max=0      -- 标准人数最大值，平时跟随实时滤波，精准人数用于校准
	local realtime_people_min=0
	local realtime_people_max=0
	local realtime_people_count=0
	local accurate_people_min=0
	local accurate_people_max=0
	local accurate_people_count=0
	local traj_x=0          -- 轨迹X坐标 cm
	local traj_y=0          -- 轨迹Y坐标 cm
	local traj_height=0     -- 轨迹高度 cm
	local traj_speed=0      -- 轨迹速度 cm/s
	local has_person=0      -- 是否有人: 0=无人, 1=有人
	local people_count=0    -- 当前标准人数，平时以实时人数滤波为主，精准人数用于验证校准
	local under_bed_people_count=0
	local target_count=0    -- 当前轨迹目标数量
	local target_ids=""     -- 当前目标ID列表，逗号分隔
	local posture_code=0    -- 主目标姿态: 0=未知/无人, 1=站立, 2=坐下, 3=躺下
	local posture_text="无人"
	local main_posture_text="无人"
	local primary_target_id=-1
	local primary_target_x=0
	local primary_target_y=0
	local primary_target_height=0
	local primary_target_speed=0
	local primary_target_posture=0
	local primary_in_bed_zone=0
	local target0_id=-1
	local target0_x=0
	local target0_y=0
	local target0_height=0
	local target0_speed=0
	local target0_posture=0
	local target0_posture_text="无人"
	local target0_in_bed_zone=0
	local target1_id=-1
	local target1_x=0
	local target1_y=0
	local target1_height=0
	local target1_speed=0
	local target1_posture=0
	local target1_posture_text="无人"
	local target1_in_bed_zone=0
	local target2_id=-1
	local target2_x=0
	local target2_y=0
	local target2_height=0
	local target2_speed=0
	local target2_posture=0
	local target2_posture_text="无人"
	local target2_in_bed_zone=0
	local target0={}
	local target1={}
	local target2={}
	
	-- 串口接收缓存
	local rx_buf=""
	
	-- 周期上报控制
	local lasttime=os.time()
	local needdup=1
	local last_dynamic_dup_time=os.time()
	local msg_seq=0
	local boot_time=os.time()
	local radar_ready=0
	local radar_frame_count=0
	local uart_bytes=0
	local last_uart_log_time=os.time()
	
	-- 人体状态变化标记
	local last_human=-1
	local last_motion=-1
	local last_people_min=-1
	local last_people_max=-1
	local last_realtime_people_count=-1
	local last_accurate_people_count=-1
	local last_body_move=-1
	local last_traj_x=-999999
	local last_traj_y=-999999
	local last_traj_height=-1
	local last_traj_speed=-999999
	local last_target_count=-1
	local last_under_bed_people_count=-1
	local last_posture_code=-1
	local people_pending_count=-1
	local people_pending_hits=0
	local accurate_pending_count=-1
	local accurate_pending_hits=0
	local target_people_count=0
	local target_people_pending_count=-1
	local target_people_pending_hits=0
	local target_clear_hits=0
	local target_constraint_active=0
	local target_constraint_count=0
	local under_bed_pending_count=-1
	local under_bed_pending_hits=0
	local posture_filters={}
	
	-- 停止平台自带的串口协议解析，改为任务手动读取
	UartStopProRecCh(1)
	PronetStopProRecCh(nid)
	
	-- 串口已由平台配置为 115200 8N1，任务直接使用
	log.info(tname,"UART1 ready")
	
	-- ============================================================
	-- 辅助函数
	-- ============================================================
	
	-- 查询table数组里面是否有某个变量
	local function CheckNameInTable(t,value)
		if type(t)~="table" then return false end
		for i,v in ipairs(t) do
			if v==value then return true end
		end
		return false
	end

	local function next_msg_id()
		msg_seq=msg_seq+1
		if msg_seq>999999 then msg_seq=1 end
		return tostring(os.time())..tostring(msg_seq)
	end

	local function now_ms()
		return tostring(os.time()).."000"
	end
	
	-- 通过CSQ获取信号强度百分比
	local function TaskGetRssi()
		local csq=mobile.csq()
		if not csq or csq<0 then return 0 end
		local r=0
		if csq>29 then r=100
		elseif csq>25 then r=90
		elseif csq>22 then r=80
		elseif csq>20 then r=70
		elseif csq>18 then r=60
		elseif csq>16 then r=50
		elseif csq>14 then r=40
		else r=0
		end
		return r
	end
	
	-- 大端序无符号16位
	local function parse_uint16_be(hi,lo)
		return hi*256+lo
	end

	-- 雷达坐标/速度使用最高位表示符号: 0=正, 1=负
	local function parse_radar_signed16_be(hi,lo)
		local val=hi*256+lo
		if val>=32768 then
			return -(val-32768)
		end
		return val
	end

	local function in_side_area(x,y)
		local ax=math.abs(x)
		return ax>=SIDE_AREA_X_MIN_ABS and ax<=SIDE_AREA_X_MAX_ABS and y>=BED_Y_MIN and y<=BED_Y_MAX
	end

	local function in_bed_zone(x,y)
		local ax=math.abs(x)
		return ax>=BED_AREA_X_MIN_ABS and ax<=SIDE_AREA_X_MAX_ABS and y>=BED_SURFACE_Y_MIN and y<=BED_Y_MAX
	end

	local function in_lower_side_area(x,y)
		return in_side_area(x,y) and y<BED_SURFACE_Y_MIN
	end

	local function is_valid_target_height(h)
		return h>=TARGET_HEIGHT_MIN_CM and h<=TARGET_HEIGHT_MAX_CM
	end

	local function posture_text_by_code(code)
		if code==1 then return "站立" end
		if code==2 then return "坐下" end
		if code==3 then return "躺下" end
		return "无人"
	end

	local function clamp_people_count(v)
		if not v or v<0 then return 0 end
		if v>PEOPLE_COUNT_MAX then return PEOPLE_COUNT_MAX end
		return v
	end

	local function clamp_under_bed_people_count(v)
		if not v or v<0 then return 0 end
		if v>UNDER_BED_PEOPLE_COUNT_MAX then return UNDER_BED_PEOPLE_COUNT_MAX end
		return v
	end

	local function accept_under_bed_people_count(candidate)
		local v=clamp_under_bed_people_count(candidate)
		local needed=UNDER_BED_COUNT_CONFIRM_FRAMES
		if v==0 then needed=UNDER_BED_COUNT_ZERO_CONFIRM_FRAMES end
		if v==under_bed_people_count then
			under_bed_pending_count=-1
			under_bed_pending_hits=0
			return true
		end
		if under_bed_pending_count==v then
			under_bed_pending_hits=under_bed_pending_hits+1
		else
			under_bed_pending_count=v
			under_bed_pending_hits=1
		end
		log.info(tname,string.format("under bed people candidate=%d stable=%d hits=%d/%d",v,under_bed_people_count,under_bed_pending_hits,needed))
		if under_bed_pending_hits>=needed then
			under_bed_people_count=v
			under_bed_pending_count=-1
			under_bed_pending_hits=0
			return true
		end
		return false
	end

	local function apply_presence_from_stable_counts()
		if people_count>0 or realtime_people_count>0 or target_count>0 then
			has_person=1
			human_present=1
		else
			has_person=0
			human_present=0
		end
	end

	local function set_standard_people_count(candidate,source)
		local v=math.max(clamp_people_count(candidate),target_people_count)
		if source=="accurate_correction" then
			v=math.max(v,accurate_people_count)
		end
		people_count=v
		people_min=v
		people_max=v
		apply_presence_from_stable_counts()
		log.info(tname,string.format("people standard=%d source=%s realtime=%d accurate=%d target=%d",people_count,tostring(source),realtime_people_count,accurate_people_count,target_people_count))
	end

	local function accept_target_people_count(candidate)
		local v=clamp_people_count(candidate)
		local needed=TARGET_COUNT_CONFIRM_FRAMES
		if v==0 then needed=TARGET_COUNT_ZERO_CONFIRM_FRAMES end
		if v==target_people_count then
			target_people_pending_count=-1
			target_people_pending_hits=0
			return true
		end
		if target_people_pending_count==v then
			target_people_pending_hits=target_people_pending_hits+1
		else
			target_people_pending_count=v
			target_people_pending_hits=1
		end
		log.info(tname,string.format("target people candidate=%d stable=%d hits=%d/%d",v,target_people_count,target_people_pending_hits,needed))
		if target_people_pending_hits>=needed then
			target_people_count=v
			target_people_pending_count=-1
			target_people_pending_hits=0
			set_standard_people_count(realtime_people_count,"target_people")
			return true
		end
		return false
	end

	local function accept_realtime_people_count(raw_min,raw_max)
		local candidate=clamp_people_count(raw_max)
		realtime_people_min=clamp_people_count(raw_min)
		realtime_people_max=clamp_people_count(raw_max)
		local needed=PEOPLE_CHANGE_CONFIRM_FRAMES
		if candidate==0 then needed=PEOPLE_ZERO_CONFIRM_FRAMES end
		if candidate==realtime_people_count then
			people_pending_count=-1
			people_pending_hits=0
			set_standard_people_count(realtime_people_count,"realtime")
			return true
		end
		if people_pending_count==candidate then
			people_pending_hits=people_pending_hits+1
		else
			people_pending_count=candidate
			people_pending_hits=1
		end
		log.info(tname,string.format("realtime people candidate=%d filtered=%d hits=%d/%d accurate=%d",candidate,realtime_people_count,people_pending_hits,needed,accurate_people_count))
		if people_pending_hits>=needed then
			realtime_people_count=candidate
			people_pending_count=-1
			people_pending_hits=0
			set_standard_people_count(realtime_people_count,"realtime_filtered")
			return true
		end
		return false
	end

	local function accept_accurate_people_count(raw_min,raw_max)
		local candidate=clamp_people_count(raw_max)
		accurate_people_min=clamp_people_count(raw_min)
		accurate_people_max=clamp_people_count(raw_max)
		accurate_people_count=candidate
		if candidate==people_count or candidate==realtime_people_count then
			accurate_pending_count=-1
			accurate_pending_hits=0
			log.info(tname,string.format("accurate people verified=%d realtime=%d standard=%d",candidate,realtime_people_count,people_count))
			return true
		end
		if accurate_pending_count==candidate then
			accurate_pending_hits=accurate_pending_hits+1
		else
			accurate_pending_count=candidate
			accurate_pending_hits=1
		end
		log.info(tname,string.format("accurate people conflict candidate=%d standard=%d realtime=%d hits=%d/%d",candidate,people_count,realtime_people_count,accurate_pending_hits,ACCURATE_CONFLICT_CONFIRM_FRAMES))
		if accurate_pending_hits>=ACCURATE_CONFLICT_CONFIRM_FRAMES then
			realtime_people_count=candidate
			set_standard_people_count(candidate,"accurate_correction")
			accurate_pending_count=-1
			accurate_pending_hits=0
			return true
		end
		return false
	end

	local function posture_by_height(h,x,y,prev_code)
		if h<=0 then return 0,"无人" end
		local margin=POSTURE_HYSTERESIS_CM
		if in_lower_side_area(x,y) then
			if h>=SIT_HEIGHT_CM then return 2,"坐下" end
			return 3,"躺下"
		end
		if in_bed_zone(x,y) and h>=BED_SIT_HEIGHT_CM then
			return 2,"坐下"
		elseif in_bed_zone(x,y) and h>=BED_HEIGHT_CM-BED_ON_TOLERANCE_CM then
			local bed_sit_line=BED_SIT_HEIGHT_CM
			if prev_code==2 then bed_sit_line=BED_SIT_HEIGHT_CM-margin end
			if prev_code==3 then bed_sit_line=BED_SIT_HEIGHT_CM+margin end
			if h>=bed_sit_line then
				return 2,"坐下"
			end
			return 3,"躺下"
		end
		local stand_line=STAND_HEIGHT_CM
		local sit_line=SIT_HEIGHT_CM
		if prev_code==1 then
			stand_line=STAND_HEIGHT_CM-margin
		elseif prev_code==2 then
			stand_line=STAND_HEIGHT_CM+margin
			sit_line=SIT_HEIGHT_CM-margin
		elseif prev_code==3 then
			sit_line=SIT_HEIGHT_CM+margin
		end
		if h>=stand_line then return 1,"站立" end
		if h>=sit_line then return 2,"坐下" end
		return 3,"躺下"
	end

	local function stable_posture_for_target(id,h,x,y)
		local key=tostring(id)
		local state=posture_filters[key]
		if not state then
			state={stable_code=0,stable_text="无人",raw_code=-1,raw_hits=0}
			posture_filters[key]=state
		end
		local raw_code,raw_text=posture_by_height(h,x,y,state.stable_code)
		if state.stable_code==0 or raw_code==state.stable_code then
			state.stable_code=raw_code
			state.stable_text=raw_text
			state.raw_code=raw_code
			state.raw_hits=0
		else
			if state.raw_code==raw_code then
				state.raw_hits=state.raw_hits+1
			else
				state.raw_code=raw_code
				state.raw_hits=1
			end
			if state.raw_hits>=POSTURE_CONFIRM_FRAMES then
				state.stable_code=raw_code
				state.stable_text=raw_text
				state.raw_hits=0
			end
		end
		return state.stable_code,posture_text_by_code(state.stable_code)
	end

	local function clear_targets()
		target_count=0
		target_ids=""
		traj_x=0
		traj_y=0
		traj_height=0
		traj_speed=0
		primary_target_id=-1
		primary_target_x=0
		primary_target_y=0
		primary_target_height=0
		primary_target_speed=0
		primary_target_posture=0
		primary_in_bed_zone=0
		posture_code=0
		posture_text="无人"
		main_posture_text="无人"
		target0_id=-1
		target0_x=0
		target0_y=0
		target0_height=0
		target0_speed=0
		target0_posture=0
		target0_posture_text="无人"
		target0_in_bed_zone=0
		target1_id=-1
		target1_x=0
		target1_y=0
		target1_height=0
		target1_speed=0
		target1_posture=0
		target1_posture_text="无人"
		target1_in_bed_zone=0
		target2_id=-1
		target2_x=0
		target2_y=0
		target2_height=0
		target2_speed=0
		target2_posture=0
		target2_posture_text="无人"
		target2_in_bed_zone=0
		target0={id=-1,x=0,y=0,height=0,speed=0,posture=0,posture_text="无人",in_bed_zone=0}
		target1={id=-1,x=0,y=0,height=0,speed=0,posture=0,posture_text="无人",in_bed_zone=0}
		target2={id=-1,x=0,y=0,height=0,speed=0,posture=0,posture_text="无人",in_bed_zone=0}
	end

	local function clear_targets_stable()
		clear_targets()
		posture_filters={}
	end

	local function set_target_slot(slot,id,x,y,h,s,pcode,ptext)
		local in_bed=in_bed_zone(x,y) and 1 or 0
		if slot==0 then
			target0_id=id
			target0_x=x
			target0_y=y
			target0_height=h
			target0_speed=s
			target0_posture=pcode
			target0_posture_text=ptext
			target0_in_bed_zone=in_bed
			target0={id=id,x=x,y=y,height=h,speed=s,posture=pcode,posture_text=ptext,in_bed_zone=in_bed}
		elseif slot==1 then
			target1_id=id
			target1_x=x
			target1_y=y
			target1_height=h
			target1_speed=s
			target1_posture=pcode
			target1_posture_text=ptext
			target1_in_bed_zone=in_bed
			target1={id=id,x=x,y=y,height=h,speed=s,posture=pcode,posture_text=ptext,in_bed_zone=in_bed}
		elseif slot==2 then
			target2_id=id
			target2_x=x
			target2_y=y
			target2_height=h
			target2_speed=s
			target2_posture=pcode
			target2_posture_text=ptext
			target2_in_bed_zone=in_bed
			target2={id=id,x=x,y=y,height=h,speed=s,posture=pcode,posture_text=ptext,in_bed_zone=in_bed}
		end
	end

	local function get_datapoint_list(param)
		if type(param)~="table" then return {} end
		if type(param.datapoint)=="table" then return param.datapoint end
		return param
	end

	local function param_wanted(names,name)
		return #names==0 or CheckNameInTable(names,name)
	end

	local function fill_param(dst,names,force_all)
		if force_all or param_wanted(names,"imei") then dst.imei=mobile.imei() end
		if force_all or param_wanted(names,"iccid") then dst.iccid=mobile.iccid() end
		if force_all or param_wanted(names,"imsi") then dst.imsi=mobile.imsi() end
		if force_all or param_wanted(names,"pver") then dst.pver=pver end
		if force_all or param_wanted(names,"fver") then dst.fver=fver end
		if force_all or param_wanted(names,"rssi") then dst.rssi=TaskGetRssi() end
		if force_all or param_wanted(names,"pele") then dst.pele=100 end
		if force_all or param_wanted(names,"human_present") then dst.human_present=human_present end
		if force_all or param_wanted(names,"motion") then dst.motion=motion end
		if force_all or param_wanted(names,"body_move") then dst.body_move=body_move end
		if force_all or param_wanted(names,"people_min") then dst.people_min=people_min end
		if force_all or param_wanted(names,"people_max") then dst.people_max=people_max end
		if force_all or param_wanted(names,"realtime_people_min") then dst.realtime_people_min=realtime_people_min end
		if force_all or param_wanted(names,"realtime_people_max") then dst.realtime_people_max=realtime_people_max end
		if force_all or param_wanted(names,"realtime_people_count") then dst.realtime_people_count=realtime_people_count end
		if force_all or param_wanted(names,"accurate_people_min") then dst.accurate_people_min=accurate_people_min end
		if force_all or param_wanted(names,"accurate_people_max") then dst.accurate_people_max=accurate_people_max end
		if force_all or param_wanted(names,"accurate_people_count") then dst.accurate_people_count=accurate_people_count end
		if force_all or param_wanted(names,"traj_x") then dst.traj_x=traj_x end
		if force_all or param_wanted(names,"traj_y") then dst.traj_y=traj_y end
		if force_all or param_wanted(names,"traj_height") then dst.traj_height=traj_height end
		if force_all or param_wanted(names,"traj_speed") then dst.traj_speed=traj_speed end
		if force_all or param_wanted(names,"has_person") then dst.has_person=has_person end
		if force_all or param_wanted(names,"people_count") then dst.people_count=people_count end
		if force_all or param_wanted(names,"under_bed_people_count") then dst.under_bed_people_count=under_bed_people_count end
		if force_all or param_wanted(names,"target_count") then dst.target_count=target_count end
		if force_all or param_wanted(names,"target_ids") then dst.target_ids=target_ids end
		if force_all or param_wanted(names,"posture_code") then dst.posture_code=posture_code end
		if force_all or param_wanted(names,"main_posture_text") then dst.main_posture_text=main_posture_text end
		if force_all or param_wanted(names,"primary_target_id") then dst.primary_target_id=primary_target_id end
		if force_all or param_wanted(names,"primary_target_x") then dst.primary_target_x=primary_target_x end
		if force_all or param_wanted(names,"primary_target_y") then dst.primary_target_y=primary_target_y end
		if force_all or param_wanted(names,"primary_target_height") then dst.primary_target_height=primary_target_height end
		if force_all or param_wanted(names,"primary_target_speed") then dst.primary_target_speed=primary_target_speed end
		if force_all or param_wanted(names,"primary_target_posture") then dst.primary_target_posture=primary_target_posture end
		if force_all or param_wanted(names,"primary_in_bed_zone") then dst.primary_in_bed_zone=primary_in_bed_zone end
		if force_all or param_wanted(names,"target0_id") then dst.target0_id=target0_id end
		if force_all or param_wanted(names,"target0_x") then dst.target0_x=target0_x end
		if force_all or param_wanted(names,"target0_y") then dst.target0_y=target0_y end
		if force_all or param_wanted(names,"target0_height") then dst.target0_height=target0_height end
		if force_all or param_wanted(names,"target0_speed") then dst.target0_speed=target0_speed end
		if force_all or param_wanted(names,"target0_posture") then dst.target0_posture=target0_posture end
		if force_all or param_wanted(names,"target0_posture_text") then dst.target0_posture_text=target0_posture_text end
		if force_all or param_wanted(names,"target0_in_bed_zone") then dst.target0_in_bed_zone=target0_in_bed_zone end
		if force_all or param_wanted(names,"target1_id") then dst.target1_id=target1_id end
		if force_all or param_wanted(names,"target1_x") then dst.target1_x=target1_x end
		if force_all or param_wanted(names,"target1_y") then dst.target1_y=target1_y end
		if force_all or param_wanted(names,"target1_height") then dst.target1_height=target1_height end
		if force_all or param_wanted(names,"target1_speed") then dst.target1_speed=target1_speed end
		if force_all or param_wanted(names,"target1_posture") then dst.target1_posture=target1_posture end
		if force_all or param_wanted(names,"target1_posture_text") then dst.target1_posture_text=target1_posture_text end
		if force_all or param_wanted(names,"target1_in_bed_zone") then dst.target1_in_bed_zone=target1_in_bed_zone end
		if force_all or param_wanted(names,"target2_id") then dst.target2_id=target2_id end
		if force_all or param_wanted(names,"target2_x") then dst.target2_x=target2_x end
		if force_all or param_wanted(names,"target2_y") then dst.target2_y=target2_y end
		if force_all or param_wanted(names,"target2_height") then dst.target2_height=target2_height end
		if force_all or param_wanted(names,"target2_speed") then dst.target2_speed=target2_speed end
		if force_all or param_wanted(names,"target2_posture") then dst.target2_posture=target2_posture end
		if force_all or param_wanted(names,"target2_posture_text") then dst.target2_posture_text=target2_posture_text end
		if force_all or param_wanted(names,"target2_in_bed_zone") then dst.target2_in_bed_zone=target2_in_bed_zone end
		if force_all or param_wanted(names,"target0") then dst.target0=target0 end
		if force_all or param_wanted(names,"target1") then dst.target1=target1 end
		if force_all or param_wanted(names,"target2") then dst.target2=target2 end
		if force_all or param_wanted(names,"radar_ready") then dst.radar_ready=radar_ready end
		if force_all or param_wanted(names,"radar_frame_count") then dst.radar_frame_count=radar_frame_count end
		if force_all or param_wanted(names,"uart_bytes") then dst.uart_bytes=uart_bytes end
	end

	local function send_dreg()
		local b={}
		b.cmd="dreg"
		b.did=next_msg_id()
		b.iccid=mobile.iccid() or ""
		b.imsi=mobile.imsi() or ""
		b.imei=mobile.imei() or ""
		b.pver=pver
		b.fver=fver
		b.rssi=TaskGetRssi()
		b.times=now_ms()
		local s=json.encode(b)
		log.info(tname,"dreg: "..s)
		PronetSetSendCh(nid,s)
	end
	
	-- ============================================================
	-- 雷达帧解析
	-- ============================================================
	
	-- 帧头常量
	local FRAME_HEAD=string.char(0x53,0x59)
	
	-- 尝试解析一帧雷达数据
	local function try_parse_frame()
		-- 找帧头 53 59
		local head_pos=string.find(rx_buf,FRAME_HEAD,1,true)
		if not head_pos then
			rx_buf=""
			return false
		end
		
		-- 丢弃帧头前无效数据
		if head_pos>1 then
			rx_buf=string.sub(rx_buf,head_pos)
		end
		
		-- 至少需要7字节才能读到长度字段
		if string.len(rx_buf)<7 then
			return false
		end
		
		-- 读取数据长度 (大端序, 第5、6字节)
		local data_len_hi=string.byte(rx_buf,5)
		local data_len_lo=string.byte(rx_buf,6)
		local data_len=data_len_hi*256+data_len_lo

		-- 防止误同步后读到异常长度，导致缓存长期卡住
		if data_len>256 then
			log.warn(tname,"invalid data_len="..data_len)
			rx_buf=string.sub(rx_buf,2)
			return true
		end
		
		-- 总帧长度: 头(2)+控制(1)+命令(1)+长度(2)+数据(n)+校验(1)+尾(2)
		local total_len=2+1+1+2+data_len+1+2
		
		-- 数据不完整，等待更多
		if string.len(rx_buf)<total_len then
			return false
		end
		
		-- 提取完整帧
		local frame=string.sub(rx_buf,1,total_len)
		rx_buf=string.sub(rx_buf,total_len+1)
		
		-- 解析字段
		local control_word=string.byte(frame,3)
		local command_word=string.byte(frame,4)
		
		-- 校验和验证
		local calc_sum=0
		for i=1,6+data_len do
			calc_sum=(calc_sum+string.byte(frame,i))%256
		end
		local recv_sum=string.byte(frame,6+data_len+1)
		
		if calc_sum~=recv_sum then
			log.warn(tname,"checksum fail calc="..calc_sum.." recv="..recv_sum)
			return false
		end
		
		-- 帧尾验证 54 43
		local tail1=string.byte(frame,6+data_len+2)
		local tail2=string.byte(frame,6+data_len+3)
		if tail1~=0x54 or tail2~=0x43 then
			log.warn(tname,"tail error")
			return false
		end

		radar_ready=1
		radar_frame_count=radar_frame_count+1
		
		-- 提取数据区字节
		local data_bytes={}
		for i=7,6+data_len do
			table.insert(data_bytes,string.byte(frame,i))
		end
		
		-- 解析雷达数据并更新变量
		-- 0x01 心跳包
		if control_word==0x01 then
			log.info(tname,"heartbeat")
		
		-- 0x80 人体存在
		elseif control_word==0x80 then
			if (command_word==0x01 or command_word==0x81) and #data_bytes>=1 then
				local raw_human=data_bytes[1]
				if raw_human==1 then
					human_present=1
					has_person=1
				else
					if people_count==0 and realtime_people_count==0 then
						clear_targets_stable()
					end
				end
				log.info(tname,"human raw="..(raw_human==1 and "有人" or "无人").." stable="..human_present)
			elseif (command_word==0x02 or command_word==0x82) and #data_bytes>=1 then
				motion=data_bytes[1]
				log.info(tname,"motion="..data_bytes[1])
			elseif (command_word==0x03 or command_word==0x83) and #data_bytes>=1 then
				body_move=data_bytes[1]
				log.info(tname,"body_move="..data_bytes[1])
			end
		
		-- 0x82 轨迹跟踪
		elseif control_word==0x82 then
			if (command_word==0x02 or command_word==0x82) and #data_bytes>=11 then
				-- 实测R60BMP1每个目标11字节: idx,size,feature,x(2),y(2),height(2),speed(2)
				local target_len=11
				local frame_target_count=math.floor(#data_bytes/target_len)
				local valid_targets={}
				local dropped_target_count=0
				local under_bed_frame_count=0
				clear_targets()
				target_clear_hits=0
				for n=0,frame_target_count-1 do
					local offset=n*target_len+1
					local tid=data_bytes[offset]
					local x=parse_radar_signed16_be(data_bytes[offset+3],data_bytes[offset+4])
					local y=parse_radar_signed16_be(data_bytes[offset+5],data_bytes[offset+6])
					local h=parse_uint16_be(data_bytes[offset+7],data_bytes[offset+8])
					local s=parse_radar_signed16_be(data_bytes[offset+9],data_bytes[offset+10])
					if is_valid_target_height(h) then
						local pcode,ptext=stable_posture_for_target(tid,h,x,y)
						local in_bed=in_bed_zone(x,y) and 1 or 0
						if in_lower_side_area(x,y) then
							under_bed_frame_count=under_bed_frame_count+1
						end
						table.insert(valid_targets,{id=tid,x=x,y=y,height=h,speed=s,posture=pcode,posture_text=ptext,in_bed_zone=in_bed})
					else
						dropped_target_count=dropped_target_count+1
						log.info(tname,string.format("drop target id=%d x=%d y=%d h=%d reason=height_out_of_range",tid,x,y,h))
					end
				end
				target_count=#valid_targets
				target_constraint_active=1
				target_constraint_count=target_count
				accept_target_people_count(target_count)
				accept_under_bed_people_count(under_bed_frame_count)
				if target_count>0 then
					has_person=1
					human_present=1
				end
				local primary_index=1
				for i=1,target_count do
					local t=valid_targets[i]
					if i==1 then
						target_ids=tostring(t.id)
					else
						target_ids=target_ids..","..tostring(t.id)
					end
					if i<=3 then
						set_target_slot(i-1,t.id,t.x,t.y,t.height,t.speed,t.posture,t.posture_text)
					end
				end
				if target_count>0 then
					local primary=valid_targets[primary_index]
					traj_x=primary.x
					traj_y=primary.y
					traj_height=primary.height
					traj_speed=primary.speed
					primary_target_id=primary.id
					primary_target_x=primary.x
					primary_target_y=primary.y
					primary_target_height=primary.height
					primary_target_speed=primary.speed
					primary_target_posture=primary.posture
					primary_in_bed_zone=primary.in_bed_zone
					posture_code=primary.posture
					posture_text=primary.posture_text
					main_posture_text=primary.posture_text
				end
				set_standard_people_count(realtime_people_count,"target_constraint")
				log.info(tname,string.format("traj raw=%d valid=%d under_bed=%d dropped=%d ids=%s main=%d x=%d y=%d h=%d v=%d posture=%s in_bed=%d",frame_target_count,target_count,under_bed_people_count,dropped_target_count,target_ids,primary_target_id,traj_x,traj_y,traj_height,traj_speed,posture_text,primary_in_bed_zone))
			elseif (command_word==0x02 or command_word==0x82) and #data_bytes==0 then
				target_clear_hits=target_clear_hits+1
				log.info(tname,string.format("traj empty hits=%d/%d",target_clear_hits,TARGET_CLEAR_CONFIRM_FRAMES))
				if target_clear_hits>=TARGET_CLEAR_CONFIRM_FRAMES then
					clear_targets_stable()
					target_constraint_active=0
					target_constraint_count=0
					accept_target_people_count(0)
					accept_under_bed_people_count(0)
				end
			end
		
		-- 0x86 人数统计
		elseif control_word==0x86 then
			if command_word==0x0A and #data_bytes>=2 then
				local raw_min=data_bytes[1]
				local raw_max=data_bytes[2]
				accept_realtime_people_count(raw_min,raw_max)
				log.info(tname,"realtime people raw min="..raw_min.." max="..raw_max.." filtered="..realtime_people_count.." accurate="..accurate_people_count)
			elseif command_word==0x0C and #data_bytes>=2 then
				local raw_min=data_bytes[1]
				local raw_max=data_bytes[2]
				accept_accurate_people_count(raw_min,raw_max)
			end
		
		-- 其他控制字，打日志
		else
			log.info(tname,"ctrl=0x"..string.format("%02X",control_word).." cmd="..command_word)
		end
		
		return true
	end
	
	-- 读取串口并解析
	local function uart_read_and_parse()
		local data=UartGetRecChAndDel(1)
		if data and string.len(data)>0 then
			uart_bytes=uart_bytes+string.len(data)
			log.info(tname,"uart recv len="..string.len(data))
			rx_buf=rx_buf..data
			-- 循环解析所有完整帧
			while try_parse_frame() do
			end
		elseif os.time()-last_uart_log_time>=30 then
			last_uart_log_time=os.time()
			log.warn(tname,"no radar uart data, frames="..radar_frame_count.." bytes="..uart_bytes)
		end
	end
	
	-- ============================================================
	-- 主循环
	-- ============================================================
	
	-- 延迟2秒等待平台完成注册
	sys.wait(2000)
	send_dreg()
	lasttime=os.time()
	last_human=human_present
	last_motion=motion
	last_body_move=body_move
	last_people_min=people_min
	last_people_max=people_max
	last_realtime_people_count=realtime_people_count
	last_accurate_people_count=accurate_people_count
	last_traj_x=traj_x
	last_traj_y=traj_y
	last_traj_height=traj_height
	last_traj_speed=traj_speed
	last_target_count=target_count
	last_posture_code=posture_code
	
	while true do
		-- 1. 读取串口数据并解析
		uart_read_and_parse()
		
		-- 2. 接收服务器下发数据
		local netr=PronetGetRecChAndDel(nid)
		if netr then
			log.info(tname,"netr: "..netr)
			local obj=json.decode(netr)
			if obj then
				if obj.cmd=="sset" then
					-- 服务器设置参数
					local rst=0
					log.info(tname,"sset received")
					
					-- 应答
					local b={}
					b.cmd="ssetbck"
					b.did=tostring(obj.did or "")
					b.rst=rst
					b.times=now_ms()
					local s=json.encode(b)
					log.info(tname,"ssetbck: "..s)
					PronetSetSendCh(nid,s)
					
				elseif obj.cmd=="sget" then
					-- 服务器获取数据点
					local p=get_datapoint_list(obj.param)
					local b={}
					b.cmd="sgetbck"
					b.did=tostring(obj.did or "")
					b.rst=0
					b.times=now_ms()
					b.param={}
					
					fill_param(b.param,p,false)
					
					local s=json.encode(b)
					log.info(tname,"sgetbck: "..s)
					PronetSetSendCh(nid,s)
					
				elseif obj.cmd=="dupbck" then
					log.info(tname,"dupbck rst="..tostring(obj.rst))
					lasttime=os.time()
				elseif obj.cmd=="dregbck" or obj.cmd=="regbck" then
					log.info(tname,"dregbck did="..tostring(obj.did))
				end
			end
		end
		
		-- 3. 检查状态变化：关键状态立即上报，体动/轨迹变化限频上报
		local now=os.time()
		local state_changed=(human_present~=last_human or motion~=last_motion or people_min~=last_people_min or people_max~=last_people_max or realtime_people_count~=last_realtime_people_count or accurate_people_count~=last_accurate_people_count or target_count~=last_target_count or posture_code~=last_posture_code)
		local dynamic_changed=(math.abs(body_move-last_body_move)>=10 or math.abs(traj_x-last_traj_x)>=20 or math.abs(traj_y-last_traj_y)>=20 or math.abs(traj_height-last_traj_height)>=20 or math.abs(traj_speed-last_traj_speed)>=10)
		if state_changed or (dynamic_changed and now-last_dynamic_dup_time>=10) then
			last_human=human_present
			last_motion=motion
			last_body_move=body_move
			last_people_min=people_min
			last_people_max=people_max
			last_realtime_people_count=realtime_people_count
			last_accurate_people_count=accurate_people_count
			last_traj_x=traj_x
			last_traj_y=traj_y
			last_traj_height=traj_height
			last_traj_speed=traj_speed
			last_target_count=target_count
			last_posture_code=posture_code
			last_dynamic_dup_time=now
			needdup=1
			log.info(tname,"state changed need dup")
		end
		
		-- 4. 周期上报 (180秒 = 3分钟, 确保5分钟内有心跳)
		if os.time()-lasttime>180 then
			needdup=1
			log.info(tname,"heartbeat timeout need dup")
		end
		
		-- 5. 主动上报 dup
		if needdup==1 and (radar_ready==1 or os.time()-boot_time>=30) then
			local b={}
			b.cmd="dup"
			b.did=next_msg_id()
			b.times=now_ms()
			b.param={}
			
			fill_param(b.param,{},true)
			
			local s=json.encode(b)
			log.info(tname,"dup: "..s)
			PronetSetSendCh(nid,s)
			needdup=0
			lasttime=os.time()
		end
		
		sys.wait(100)
	end
end
