--[[
============================================================
  R60BMP1 top-mount radar door-exit people-count task

  Purpose: parse ceiling-mounted radar frames, maintain confirmed
  internal tracks, and keep the existing cloud datapoint contract.

  Added: log every valid raw frame as HEX to local DTU log,
  for protocol verification.

  Count rule:
  - Confirmed internal tracks are the people_count source.
  - A confirmed track is removed only after it returns from the room core
    to the calibrated door-label zone and then disappears.
  - 0x86 accurate/realtime counts are reference evidence used only to
    reconcile stale door-side holding tracks; they never create people.
  - 0x86/0x14 door-out events shorten exit confirmation but remove at most
    one eligible door-side track.
  - Raw target_count is debug only, never people_count.
  - Only confirmed tracks inside the configured top-mount area count as people.
  - Raw targets outside the software area remain visible for interference diagnosis.
  - The software door-label zone uses the recorded top-mount reference and
    must be checked against final exit coordinates before production use.
  - Confirmed tracks are held through static radar loss, then removed
    only by calibrated-zone, stale, or fallback rules.
  - target0/target1/target2 come directly from the latest valid 0x82/0x02
    payload and use the existing side-mount cloud object contract.
  - primary_target/traj fields remain the confirmed internal-track report.
  - uart_bytes reports latest raw UART bytes as HEX text (still uploaded).
============================================================
]]

function
	local tname="RADAR_TOP_DOOR_EXIT"
	local nid=1
	local pver,fver="Radar_Top_Door_Exit_1.0","FW_1.0.0"

	-- ============== 顶装现场标定参数（集中在此处调整） ==============
	-- 房间坐标：雷达位于房间中心，X 对应 330cm 宽，Y 对应 500cm 长。
	local AREA_X_MIN=-165
	local AREA_X_MAX=165
	local AREA_Y_MIN=-250
	local AREA_Y_MAX=250

	-- 软件门标签：取自 2026-07-12 顶装实物记录中的自动门标签参考。
	-- 标签中心约 (-175,0)cm、矩形 150x200cm，对应以下边界。
	-- 正式部署前仍应以“出门时轨迹最后坐标”复核并微调。
	local EXIT_ZONE_ENABLED=true
	local EXIT_ZONE_X_MIN=-250
	local EXIT_ZONE_X_MAX=-100
	local EXIT_ZONE_Y_MIN=-100
	local EXIT_ZONE_Y_MAX=100
	local EXIT_ARM_HITS=2
	local EXIT_REQUIRE_RECENT_MOVE_SEC=6
	local DOOR_OUT_EVENT_TTL_SEC=6
	local REFERENCE_DEFICIT_CONFIRM_SEC=5
	local MAX_EXPECTED_PEOPLE=5

	-- 顶装不存在已确认的“下部区域”方向；默认关闭该区域的宽匹配和快速丢失退出。
	-- 如现场确认需要特殊丢失区，再设置边界并开启。
	local LOWER_AREA_ENABLED=false
	local LOWER_AREA_Y_MAX=0

	local TARGET_HEIGHT_MIN_CM=20
	local TARGET_HEIGHT_MAX_CM=245
	local TRACK_TABLE_MAX=32
	local TRACK_RECORD_LEN=11
	local CLOUD_TRACK_SLOTS=3

	local ID_MATCH_MAX_JUMP_CM=150
	local MATCH_DIST_CM=90
	local LOWER_MATCH_DIST_CM=110
	local SPLIT_SUPPRESS_CM=45
	local LOWER_SPLIT_SUPPRESS_CM=110

	local CONFIRM_HITS=4
	local CONFIRM_MOVE_CM=60
	local CONFIRM_MOVING_HITS=2
	local MOVING_SPEED_CM_S=10
	local MOVING_STEP_CM=15
	local ACTIVE_TO_HOLD_SEC=2
	local EXIT_GRACE_SEC=3
	local LOWER_MOVING_LOST_EXIT_SEC=8
	local CAND_MISS_TTL_SEC=2
	local FALLBACK_ZERO_SECONDS=15
	local RADAR_STALE_SECONDS=30
	local Count={
		cfg={
			source="track", -- this independent variant counts confirmed tracks
			accurate_stale_sec=90,
			accurate_warn_sec=130
		},
		state={
			valid=0,
			pending=1,
			pending_reason="waiting_first_accurate",
			track_count=0,
			realtime_received=0,
			door_event=-1,
			door_event_count=0,
			door_event_at=0,
			warned_stale=0,
			warned_waiting=0
		}
	}

	-- ============== 状态变量 ==============
	local human_present=0
	local motion=0
	local body_move=0
	local has_person=0
	local people_count=0
	local people_min=0
	local people_max=0
	local realtime_people_min=0
	local realtime_people_max=0
	local realtime_people_count=0
	local accurate_people_min=0
	local accurate_people_max=0
	local accurate_people_count=0
	local accurate_updated_at=0

	local target_count=0
	local target_ids=""
	local primary_target_id=-1
	local primary_target_x=0
	local primary_target_y=0
	local primary_target_height=0
	local primary_target_speed=0
	local traj_x=0
	local traj_y=0
	local traj_height=0
	local traj_speed=0
	local raw_targets={
		{id=-1,size=0,feature=0,x=0,y=0,height=0,speed=0},
		{id=-1,size=0,feature=0,x=0,y=0,height=0,speed=0},
		{id=-1,size=0,feature=0,x=0,y=0,height=0,speed=0}
	}

	local radar_ready=0
	local radar_frame_count=0
	local uart_bytes=""
	local uart_byte_count=0
	local rx_buf=""
	local uid_seq=0
	local tracks={}
	local all_clear_since=0
	local radar_stale_cleared=0
	local door_out_pending_at=0
	local reference_deficit_since=0

	local last_valid_frame_time=os.time()
	local last_track_frame_time=0
	local last_uart_log_time=os.time()
	local lasttime=os.time()
	local boot_time=os.time()
	local msg_seq=0
	local needdup=1
	local force_initial_dup=1

	local last_human=-1
	local last_motion=-1
	local last_body_move=-1
	local last_has_person=-1
	local last_people_count=-1
	local last_target_count=-1
	local last_realtime_people_count=-1
	local last_accurate_people_count=-1
	local last_primary_target_id=-999
	local last_radar_frame_count=-1
	local last_traj_x=-999999
	local last_traj_y=-999999
	local last_traj_height=-999999
	local last_traj_speed=-999999
	local last_uart_bytes=""

	UartStopProRecCh(1)
	PronetStopProRecCh(nid)
	log.info(tname,"start")

	-- ============== 辅助函数 ==============
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

	local function TaskGetRssi()
		local csq=mobile.csq()
		if not csq or csq<0 then return 0 end
		if csq>29 then return 100 end
		if csq>25 then return 90 end
		if csq>22 then return 80 end
		if csq>20 then return 70 end
		if csq>18 then return 60 end
		if csq>16 then return 50 end
		if csq>14 then return 40 end
		return 0
	end

	local function parse_uint16_be(hi,lo)
		return hi*256+lo
	end

	local function parse_radar_signed16_be(hi,lo)
		local val=hi*256+lo
		if val>=32768 then return -(val-32768) end
		return val
	end

	local function bytes_to_hex(data)
		local parts={}
		for i=1,string.len(data) do
			parts[i]=string.format("%02X",string.byte(data,i))
		end
		return table.concat(parts," ")
	end

	local function normalize_people_count(v)
		if not v or v<0 then return 0 end
		return v
	end

	function Count.update_has_person()
		if (Count.state.valid==1 and people_count>0) or human_present==1 then has_person=1 else has_person=0 end
	end

	function Count.accurate_age(now)
		if accurate_updated_at<=0 then return -1 end
		local age=(now or os.time())-accurate_updated_at
		if age<0 then return 0 end
		return age
	end

	function Count.set_pending(reason,now)
		if Count.cfg.source~="accurate" then return end
		local changed=(Count.state.pending~=1)
		Count.state.pending=1
		if changed or Count.state.pending_reason=="" then
			Count.state.pending_reason=reason or "waiting_accurate"
		end
		if changed then
			log.info(tname,"people pending reason="..tostring(Count.state.pending_reason).." at="..tostring(now or os.time()))
			needdup=1
		end
	end

	function Count.apply_final(value,source,reason)
		people_count=normalize_people_count(value)
		people_min=people_count
		people_max=people_count
		Count.state.valid=1
		Count.state.pending=0
		Count.state.pending_reason=""
		Count.state.warned_stale=0
		Count.state.warned_waiting=0
		Count.update_has_person()
		needdup=1
		log.info(tname,string.format("people accept source=%s count=%d reason=%s",tostring(source),people_count,tostring(reason)))
	end

	function Count.accept_accurate(raw_min,raw_max,now)
		accurate_people_min=normalize_people_count(raw_min)
		accurate_people_max=normalize_people_count(raw_max)
		accurate_people_count=accurate_people_max
		accurate_updated_at=now
		Count.state.warned_stale=0
		Count.state.warned_waiting=0
		if accurate_people_min==accurate_people_max then
			if Count.cfg.source=="accurate" then
				Count.apply_final(accurate_people_max,"accurate","min_equals_max")
				if Count.state.realtime_received==1 and (realtime_people_min~=realtime_people_max or realtime_people_count~=people_count) then
					Count.set_pending("realtime_diff_after_accurate",now)
				end
			end
		else
			Count.set_pending("accurate_range",now)
			log.warn(tname,string.format("accurate range pending min=%d max=%d",accurate_people_min,accurate_people_max))
			needdup=1
		end
	end

	function Count.record_door_event(value,now)
		if value~=0 and value~=1 then
			log.warn(tname,"invalid door event="..tostring(value))
			return
		end
		Count.state.door_event=value
		Count.state.door_event_count=Count.state.door_event_count+1
		Count.state.door_event_at=now
		if value==1 then door_out_pending_at=now else door_out_pending_at=0 end
		Count.state.pending_reason=(value==0) and "door_in" or "door_out"
		Count.set_pending(Count.state.pending_reason,now)
		needdup=1
		log.info(tname,string.format("door event=%s count=%d",value==0 and "in" or "out",Count.state.door_event_count))
	end

	function Count.update_freshness(now)
		if Count.cfg.source=="track" then
			Count.state.valid=radar_ready
			Count.state.pending=0
			Count.state.pending_reason=""
			Count.update_has_person()
			return
		end
		if accurate_updated_at<=0 then
			Count.state.valid=0
			Count.state.pending=1
			Count.update_has_person()
			if now-boot_time>=Count.cfg.accurate_stale_sec and Count.state.warned_waiting==0 then
				Count.state.warned_waiting=1
				log.warn(tname,"still waiting first accurate people frame")
				needdup=1
			end
			return
		end
		local age=Count.accurate_age(now)
		if age>Count.cfg.accurate_stale_sec and Count.state.valid~=0 then
			Count.state.valid=0
			Count.state.pending=1
			Count.state.pending_reason="accurate_stale"
			Count.update_has_person()
			log.warn(tname,"accurate people stale age="..tostring(age))
			needdup=1
		end
		if age>Count.cfg.accurate_warn_sec and Count.state.warned_stale==0 then
			Count.state.warned_stale=1
			log.warn(tname,"accurate people missing two cycles age="..tostring(age))
		end
	end

	function Count.reset(reason)
		people_count=0
		people_min=0
		people_max=0
		Count.state.valid=0
		Count.state.pending=1
		Count.state.pending_reason=reason or "reset"
		Count.state.track_count=0
		Count.state.realtime_received=0
		Count.state.door_event=-1
		Count.state.door_event_at=0
		door_out_pending_at=0
		reference_deficit_since=0
		Count.state.warned_stale=0
		Count.state.warned_waiting=0
		Count.update_has_person()
		needdup=1
	end

	local function dist_xy(ax,ay,bx,by)
		local dx=ax-bx
		local dy=ay-by
		return math.sqrt(dx*dx+dy*dy)
	end

	local function in_area(x,y)
		return x>=AREA_X_MIN and x<=AREA_X_MAX and y>=AREA_Y_MIN and y<=AREA_Y_MAX
	end

	local function in_exit_zone(x,y)
		return EXIT_ZONE_ENABLED and x>=EXIT_ZONE_X_MIN and x<=EXIT_ZONE_X_MAX and y>=EXIT_ZONE_Y_MIN and y<=EXIT_ZONE_Y_MAX
	end

	local function is_countable_track(t)
		return t.confirmed and (t.state=="active" or t.state=="holding") and in_area(t.x,t.y)
	end

	local function in_room_core(x,y)
		return in_area(x,y) and not in_exit_zone(x,y)
	end

	local function exit_zone_distance(x,y)
		local dx=0
		local dy=0
		if x<EXIT_ZONE_X_MIN then dx=EXIT_ZONE_X_MIN-x elseif x>EXIT_ZONE_X_MAX then dx=x-EXIT_ZONE_X_MAX end
		if y<EXIT_ZONE_Y_MIN then dy=EXIT_ZONE_Y_MIN-y elseif y>EXIT_ZONE_Y_MAX then dy=y-EXIT_ZONE_Y_MAX end
		return math.sqrt(dx*dx+dy*dy)
	end

	local function module_reference_count(now)
		if accurate_updated_at>0 and now-accurate_updated_at<=Count.cfg.accurate_stale_sec and accurate_people_min==accurate_people_max then
			return accurate_people_max,"accurate"
		end
		if Count.state.realtime_received==1 and realtime_people_min==realtime_people_max then
			return realtime_people_max,"realtime"
		end
		return nil,"none"
	end

	local function in_lower_area(x,y)
		return LOWER_AREA_ENABLED and x>=AREA_X_MIN and x<=AREA_X_MAX and y>=AREA_Y_MIN and y<LOWER_AREA_Y_MAX
	end

	local function match_dist(ax,ay,bx,by)
		if in_lower_area(ax,ay) and in_lower_area(bx,by) then return LOWER_MATCH_DIST_CM end
		return MATCH_DIST_CM
	end

	local function split_dist(ax,ay,bx,by)
		if in_lower_area(ax,ay) and in_lower_area(bx,by) then return LOWER_SPLIT_SUPPRESS_CM end
		return SPLIT_SUPPRESS_CM
	end

	local function is_valid_height(h)
		return h>=TARGET_HEIGHT_MIN_CM and h<=TARGET_HEIGHT_MAX_CM
	end

	local function get_datapoint_list(param)
		if type(param)~="table" then return {} end
		if type(param.datapoint)=="table" then return param.datapoint end
		return param
	end

	local function param_wanted(names,name)
		return #names==0 or CheckNameInTable(names,name)
	end

	local function clear_internal_track_report()
		target_ids=""
		primary_target_id=-1
		primary_target_x=0
		primary_target_y=0
		primary_target_height=0
		primary_target_speed=0
		traj_x=0
		traj_y=0
		traj_height=0
		traj_speed=0
	end

	local function clear_raw_targets()
		target_count=0
		for i=1,CLOUD_TRACK_SLOTS do
			local t=raw_targets[i]
			t.id=-1
			t.size=0
			t.feature=0
			t.x=0
			t.y=0
			t.height=0
			t.speed=0
		end
	end

	local function target_datapoint(t)
		local has_target=t.id and t.id>=0
		return {
			id=has_target and t.id or -1,
			x=has_target and t.x or 0,
			y=has_target and t.y or 0,
			height=has_target and t.height or 0,
			speed=has_target and t.speed or 0,
			posture=0,
			posture_text=has_target and "未知" or "无人",
			in_bed_zone=0
		}
	end

	local function remove_track(index,reason)
		local t=tracks[index]
		if t then
			log.info(tname,string.format("remove uid=%d reason=%s x=%d y=%d state=%s",t.uid,tostring(reason),t.x,t.y,tostring(t.state)))
		end
		table.remove(tracks,index)
	end

	local function recompute_people_count()
		local n=0
		for i=1,#tracks do
			local t=tracks[i]
			if is_countable_track(t) then n=n+1 end
		end
		Count.state.track_count=normalize_people_count(n)
		if Count.cfg.source=="track" then
			people_count=Count.state.track_count
			people_min=people_count
			people_max=people_count
			Count.state.valid=radar_ready
			Count.state.pending=0
			Count.state.pending_reason=""
		end
		Count.update_has_person()
	end

	local function refresh_track_report()
		clear_internal_track_report()
		local best=nil
		for i=1,#tracks do
			local t=tracks[i]
			if is_countable_track(t) then
				if target_ids=="" then target_ids=tostring(t.uid) else target_ids=target_ids..","..tostring(t.uid) end
				if not best then
					best=t
				elseif t.state=="active" and best.state~="active" then
					best=t
				elseif t.state==best.state and t.hits>best.hits then
					best=t
				end
			end
		end
		if best then
			primary_target_id=best.uid
			primary_target_x=best.last_x or best.x
			primary_target_y=best.last_y or best.y
			primary_target_height=best.height
			primary_target_speed=best.speed
			traj_x=primary_target_x
			traj_y=primary_target_y
			traj_height=primary_target_height
			traj_speed=primary_target_speed
		end
	end

	local function clear_all(reason)
		tracks={}
		Count.state.track_count=0
		if Count.cfg.source=="track" or reason=="radar_stale" then Count.reset(reason) else Count.update_has_person() end
		clear_internal_track_report()
		log.info(tname,"clear all reason="..tostring(reason))
	end

	local function try_confirm(t)
		if t.confirmed then return end
		if t.hits>=CONFIRM_HITS and t.max_disp>=CONFIRM_MOVE_CM and t.moving_hits>=CONFIRM_MOVING_HITS then
			t.confirmed=true
			t.state="active"
			log.info(tname,string.format("confirm uid=%d hits=%d disp=%d moving=%d",t.uid,t.hits,t.max_disp,t.moving_hits))
		end
	end

	local function bind_detection(t,d,now)
		d.matched=true
		t.matched=true
		local old_x=t.x
		local old_y=t.y
		t.x=math.floor((t.x+d.x)/2+0.5)
		t.y=math.floor((t.y+d.y)/2+0.5)
		t.height=d.height
		t.speed=d.speed
		t.rid=d.rid
		t.hits=t.hits+1
		t.last_seen=now
		local now_in_exit=in_exit_zone(t.x,t.y)
		if now_in_exit then
			t.door_hits=(t.door_hits or 0)+1
			t.last_door_seen=now
			if t.seen_room_core and t.door_hits>=EXIT_ARM_HITS then t.door_armed=true end
		elseif in_room_core(t.x,t.y) then
			t.seen_room_core=true
			t.door_hits=0
			t.door_armed=false
		end
		t.in_exit=now_in_exit
		if in_area(t.x,t.y) then
			t.last_x=t.x
			t.last_y=t.y
		end
		local step=dist_xy(old_x,old_y,t.x,t.y)
		if math.abs(t.speed)>=MOVING_SPEED_CM_S or step>=MOVING_STEP_CM then
			t.moving_hits=t.moving_hits+1
			t.last_moving_seen=now
		end
		local disp=dist_xy(t.x,t.y,t.birth_x,t.birth_y)
		if disp>t.max_disp then t.max_disp=math.floor(disp+0.5) end
		try_confirm(t)
		if t.confirmed then t.state="active" end
	end

	local function suppress_split(d)
		for i=1,#tracks do
			local t=tracks[i]
			local sd=split_dist(t.x,t.y,d.x,d.y)
			if dist_xy(t.x,t.y,d.x,d.y)<=sd then return true end
		end
		return false
	end

	local function create_track(d,now)
		if #tracks>=TRACK_TABLE_MAX then return end
		if suppress_split(d) then return end
		uid_seq=uid_seq+1
		local t={
			uid=uid_seq,rid=d.rid,x=d.x,y=d.y,height=d.height,speed=d.speed,
			birth_x=d.x,birth_y=d.y,max_disp=0,hits=1,moving_hits=0,
			first_seen=now,last_seen=now,last_moving_seen=0,
			state="candidate",confirmed=false,matched=true,in_exit=in_exit_zone(d.x,d.y),
			seen_room_core=in_room_core(d.x,d.y),door_armed=false,
			door_hits=in_exit_zone(d.x,d.y) and 1 or 0,last_door_seen=0
		}
		if t.in_exit then t.last_door_seen=now end
		if math.abs(d.speed)>=MOVING_SPEED_CM_S then
			t.moving_hits=1
			t.last_moving_seen=now
		end
		if in_area(d.x,d.y) then
			t.last_x=d.x
			t.last_y=d.y
		end
		table.insert(tracks,t)
		d.matched=true
		log.info(tname,string.format("new uid=%d rid=%d x=%d y=%d h=%d",t.uid,t.rid,t.x,t.y,t.height))
	end

	local function on_track_frame(detections,now)
		last_valid_frame_time=now
		radar_stale_cleared=0
		for i=1,#tracks do tracks[i].matched=false end

		for i=1,#detections do
			local d=detections[i]
			local best=nil
			local best_dist=ID_MATCH_MAX_JUMP_CM+1
			for j=1,#tracks do
				local t=tracks[j]
				if not t.matched and t.rid==d.rid then
					local dd=dist_xy(t.x,t.y,d.x,d.y)
					if dd<best_dist then best=t; best_dist=dd end
				end
			end
			if best and best_dist<=ID_MATCH_MAX_JUMP_CM then bind_detection(best,d,now) end
		end

		for i=1,#detections do
			local d=detections[i]
			if not d.matched then
				local best=nil
				local best_dist=999999
				for j=1,#tracks do
					local t=tracks[j]
					if not t.matched then
						local md=match_dist(t.x,t.y,d.x,d.y)
						local dd=dist_xy(t.x,t.y,d.x,d.y)
						if dd<=md and dd<best_dist then best=t; best_dist=dd end
					end
				end
				if best then bind_detection(best,d,now) end
			end
		end

		for i=1,#detections do
			if not detections[i].matched then create_track(detections[i],now) end
		end
	end

	local function update_fallback_zero(now)
		local accurate_ok=(accurate_people_max==0) or (accurate_updated_at>0 and now-accurate_updated_at>Count.cfg.accurate_stale_sec)
		local all_clear=(human_present==0 and realtime_people_max==0 and accurate_ok)
		if all_clear then
			if all_clear_since==0 then all_clear_since=now end
			if now-all_clear_since>=FALLBACK_ZERO_SECONDS then
				clear_all("fallback_zero")
				all_clear_since=now
			end
		else
			all_clear_since=0
		end
	end

	local function confirmed_track_count()
		local n=0
		for i=1,#tracks do
			local t=tracks[i]
			if is_countable_track(t) then n=n+1 end
		end
		return n
	end

	local function door_exit_reason(t,now,gone)
		if not t.confirmed or t.state~="holding" or not t.seen_room_core or not t.in_exit then return nil end
		local event_recent=door_out_pending_at>0 and now-door_out_pending_at<=DOOR_OUT_EVENT_TTL_SEC
		if event_recent and gone>=ACTIVE_TO_HOLD_SEC then return "door_event_out" end
		local moving_recent=t.last_moving_seen>0 and now-t.last_moving_seen<=EXIT_REQUIRE_RECENT_MOVE_SEC
		if t.door_armed and t.door_hits>=EXIT_ARM_HITS and moving_recent and gone>=EXIT_GRACE_SEC then
			return "door_zone_exit"
		end
		return nil
	end

	local function reconcile_reference_deficit(now)
		local current=confirmed_track_count()
		local reference,source=module_reference_count(now)
		if not reference and current>MAX_EXPECTED_PEOPLE then
			reference=MAX_EXPECTED_PEOPLE
			source="configured_max"
		end
		if not reference or current<=reference then
			reference_deficit_since=0
			return
		end
		if reference_deficit_since==0 then
			reference_deficit_since=now
			log.warn(tname,string.format("track/reference mismatch track=%d ref=%d source=%s",current,reference,source))
			return
		end
		if now-reference_deficit_since<REFERENCE_DEFICIT_CONFIRM_SEC then return end

		local excess=current-reference
		local removed=0
		while excess>0 do
			local best_index=nil
			local best_score=-999999
			for i=1,#tracks do
				local t=tracks[i]
				if t.confirmed and t.state=="holding" and t.seen_room_core and (t.door_armed or t.in_exit) then
					local score=(t.in_exit and 100000 or 0)+(now-t.last_seen)*100-exit_zone_distance(t.x,t.y)
					if score>best_score then best_score=score; best_index=i end
				end
			end
			if not best_index then break end
			remove_track(best_index,"reference_door_prune_"..tostring(source))
			removed=removed+1
			excess=excess-1
		end
		if removed>0 then
			log.info(tname,string.format("reference prune removed=%d target=%d source=%s",removed,reference,source))
			reference_deficit_since=0
		else
			reference_deficit_since=now
			log.warn(tname,"reference mismatch has no eligible door-side holding track")
		end
	end

	local function maintain_tracks(now)
		if door_out_pending_at>0 and now-door_out_pending_at>DOOR_OUT_EVENT_TTL_SEC then door_out_pending_at=0 end
		if last_track_frame_time>0 and now-last_track_frame_time>=ACTIVE_TO_HOLD_SEC then
			clear_raw_targets()
		end
		for i=#tracks,1,-1 do
			local t=tracks[i]
			local gone=now-t.last_seen
			if not t.confirmed and gone>=CAND_MISS_TTL_SEC then
				remove_track(i,"candidate_timeout")
			elseif t.confirmed and t.state=="active" and gone>=ACTIVE_TO_HOLD_SEC then
				t.state="holding"
			elseif t.confirmed and t.state=="holding" then
				local exit_reason=door_exit_reason(t,now,gone)
				if exit_reason then
					remove_track(i,exit_reason)
					if exit_reason=="door_event_out" then door_out_pending_at=0 end
				elseif in_lower_area(t.last_x or t.x,t.last_y or t.y) and t.last_moving_seen>=t.last_seen-1 and gone>=LOWER_MOVING_LOST_EXIT_SEC then
					remove_track(i,"lower_moving_lost")
				end
			end
		end
		reconcile_reference_deficit(now)
		recompute_people_count()
		update_fallback_zero(now)
		if now-last_valid_frame_time>=RADAR_STALE_SECONDS then
			radar_ready=0
			if radar_stale_cleared==0 then
				human_present=0
				motion=0
				body_move=0
				realtime_people_min=0
				realtime_people_max=0
				realtime_people_count=0
				accurate_people_min=0
				accurate_people_max=0
				accurate_people_count=0
				accurate_updated_at=0
				clear_all("radar_stale")
				clear_raw_targets()
				radar_stale_cleared=1
			end
		else
			Count.update_freshness(now)
		end
		refresh_track_report()
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
		if force_all or param_wanted(names,"has_person") then dst.has_person=has_person end
		if force_all or param_wanted(names,"motion") then dst.motion=motion end
		if force_all or param_wanted(names,"body_move") then dst.body_move=body_move end
		if force_all or param_wanted(names,"people_count") then dst.people_count=people_count end
		if force_all or param_wanted(names,"people_min") then dst.people_min=people_min end
		if force_all or param_wanted(names,"people_max") then dst.people_max=people_max end
		if force_all or param_wanted(names,"people_count_valid") then dst.people_count_valid=Count.state.valid end
		if force_all or param_wanted(names,"people_count_pending") then dst.people_count_pending=Count.state.pending end
		if force_all or param_wanted(names,"people_count_source") then dst.people_count_source=Count.cfg.source end
		if force_all or param_wanted(names,"track_people_count") then dst.track_people_count=Count.state.track_count end
		if force_all or param_wanted(names,"realtime_people_min") then dst.realtime_people_min=realtime_people_min end
		if force_all or param_wanted(names,"realtime_people_max") then dst.realtime_people_max=realtime_people_max end
		if force_all or param_wanted(names,"realtime_people_count") then dst.realtime_people_count=realtime_people_count end
		if force_all or param_wanted(names,"accurate_people_min") then dst.accurate_people_min=accurate_people_min end
		if force_all or param_wanted(names,"accurate_people_max") then dst.accurate_people_max=accurate_people_max end
		if force_all or param_wanted(names,"accurate_people_count") then dst.accurate_people_count=accurate_people_count end
		if force_all or param_wanted(names,"accurate_age_sec") then dst.accurate_age_sec=Count.accurate_age(os.time()) end
		if force_all or param_wanted(names,"door_event") then dst.door_event=Count.state.door_event end
		if force_all or param_wanted(names,"door_event_count") then dst.door_event_count=Count.state.door_event_count end
		if force_all or param_wanted(names,"door_event_at") then dst.door_event_at=Count.state.door_event_at end
		if force_all or param_wanted(names,"target_count") then dst.target_count=target_count end
		if force_all or param_wanted(names,"target_ids") then dst.target_ids=target_ids end
		if force_all or param_wanted(names,"primary_target_id") then dst.primary_target_id=primary_target_id end
		if force_all or param_wanted(names,"primary_target_x") then dst.primary_target_x=primary_target_x end
		if force_all or param_wanted(names,"primary_target_y") then dst.primary_target_y=primary_target_y end
		if force_all or param_wanted(names,"primary_target_height") then dst.primary_target_height=primary_target_height end
		if force_all or param_wanted(names,"primary_target_speed") then dst.primary_target_speed=primary_target_speed end
		if force_all or param_wanted(names,"traj_x") then dst.traj_x=traj_x end
		if force_all or param_wanted(names,"traj_y") then dst.traj_y=traj_y end
		if force_all or param_wanted(names,"traj_height") then dst.traj_height=traj_height end
		if force_all or param_wanted(names,"traj_speed") then dst.traj_speed=traj_speed end
		if force_all or param_wanted(names,"target0") then dst.target0=target_datapoint(raw_targets[1]) end
		if force_all or param_wanted(names,"target1") then dst.target1=target_datapoint(raw_targets[2]) end
		if force_all or param_wanted(names,"target2") then dst.target2=target_datapoint(raw_targets[3]) end
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
		PronetSetSendCh(nid,json.encode(b))
	end

	local FRAME_HEAD=string.char(0x53,0x59)

	local function parse_track_payload(data_bytes,now)
		local detections={}
		clear_raw_targets()
		if #data_bytes==0 then
			last_track_frame_time=now
			on_track_frame(detections,now)
			return
		end
		if (#data_bytes%TRACK_RECORD_LEN)~=0 then
			log.warn(tname,"bad track len="..#data_bytes)
			return
		end
		last_track_frame_time=now
		target_count=math.floor(#data_bytes/TRACK_RECORD_LEN)
		for n=0,target_count-1 do
			local o=n*TRACK_RECORD_LEN+1
			local rid=data_bytes[o]
			local target_size=data_bytes[o+1]
			local target_feature=data_bytes[o+2]
			local x=parse_radar_signed16_be(data_bytes[o+3],data_bytes[o+4])
			local y=parse_radar_signed16_be(data_bytes[o+5],data_bytes[o+6])
			local h=parse_uint16_be(data_bytes[o+7],data_bytes[o+8])
			local s=parse_radar_signed16_be(data_bytes[o+9],data_bytes[o+10])
			if n<CLOUD_TRACK_SLOTS then
				local raw_target=raw_targets[n+1]
				raw_target.id=rid
				raw_target.size=target_size
				raw_target.feature=target_feature
				raw_target.x=x
				raw_target.y=y
				raw_target.height=h
				raw_target.speed=s
			end
			if is_valid_height(h) then
				table.insert(detections,{rid=rid,size=target_size,feature=target_feature,x=x,y=y,height=h,speed=s,matched=false})
			end
		end
		on_track_frame(detections,now)
	end

	local function try_parse_frame()
		local head_pos=string.find(rx_buf,FRAME_HEAD,1,true)
		if not head_pos then
			-- Preserve a trailing 0x53 because the 0x53 0x59 header may be
			-- split across two UART reads.
			if string.len(rx_buf)>0 and string.byte(rx_buf,string.len(rx_buf))==0x53 then
				rx_buf=string.char(0x53)
			else
				rx_buf=""
			end
			return false
		end
		if head_pos>1 then rx_buf=string.sub(rx_buf,head_pos) end
		if string.len(rx_buf)<7 then return false end
		local data_len=string.byte(rx_buf,5)*256+string.byte(rx_buf,6)
		if data_len>256 then rx_buf=string.sub(rx_buf,2); return true end
		local total_len=2+1+1+2+data_len+1+2
		if string.len(rx_buf)<total_len then return false end
		local frame=string.sub(rx_buf,1,total_len)

		-- 校验和
		local calc_sum=0
		for i=1,6+data_len do calc_sum=(calc_sum+string.byte(frame,i))%256 end
		local recv_sum=string.byte(frame,6+data_len+1)
		if calc_sum~=recv_sum then
			log.warn(tname,"checksum fail")
			rx_buf=string.sub(rx_buf,2)
			return true
		end
		-- 帧尾
		if string.byte(frame,6+data_len+2)~=0x54 or string.byte(frame,6+data_len+3)~=0x43 then
			log.warn(tname,"tail fail")
			rx_buf=string.sub(rx_buf,2)
			return true
		end
		-- Consume exactly one complete validated frame. Any following bytes
		-- remain buffered and are parsed by the caller's loop.
		rx_buf=string.sub(rx_buf,total_len+1)

	-- 只在校验和与帧尾均通过后更新云端原始串口数据点。
	-- 这样 uart_bytes 始终是一条完整雷达帧，而不是半帧或多帧粘包。
	uart_bytes=bytes_to_hex(frame)
	log.info(tname, "raw_frame: " .. uart_bytes)

		radar_ready=1
		radar_frame_count=radar_frame_count+1
		last_valid_frame_time=os.time()
		local ctrl=string.byte(frame,3)
		local cmd=string.byte(frame,4)
		local data_bytes={}
		for i=7,6+data_len do table.insert(data_bytes,string.byte(frame,i)) end

		if ctrl==0x80 then
			if (cmd==0x01 or cmd==0x81) and #data_bytes>=1 then
				human_present=(data_bytes[1]==1) and 1 or 0
			elseif (cmd==0x02 or cmd==0x82) and #data_bytes>=1 then
				motion=data_bytes[1]
			elseif (cmd==0x03 or cmd==0x83) and #data_bytes>=1 then
				body_move=data_bytes[1]
			end
		elseif ctrl==0x82 then
			if cmd==0x02 or cmd==0x82 then parse_track_payload(data_bytes,os.time()) end
		elseif ctrl==0x86 then
			if cmd==0x0A and #data_bytes>=2 then
				local previous_realtime=realtime_people_count
				realtime_people_min=normalize_people_count(data_bytes[1])
				realtime_people_max=normalize_people_count(data_bytes[2])
				realtime_people_count=realtime_people_max
				Count.state.realtime_received=1
				if Count.cfg.source=="accurate" and (Count.state.valid==0 or realtime_people_min~=realtime_people_max or realtime_people_count~=people_count) then
					Count.set_pending(previous_realtime~=realtime_people_count and "realtime_change" or "realtime_diff",os.time())
				end
			elseif cmd==0x0C and #data_bytes>=2 then
				Count.accept_accurate(data_bytes[1],data_bytes[2],os.time())
			elseif cmd==0x14 and #data_bytes>=1 then
				Count.record_door_event(data_bytes[1],os.time())
			end
		end
		return true
	end

	local function uart_read_and_parse()
		local data=UartGetRecChAndDel(1)
		if data and string.len(data)>0 then
			uart_byte_count=uart_byte_count+string.len(data)
			rx_buf=rx_buf..data
			while try_parse_frame() do end
		elseif os.time()-last_uart_log_time>=30 then
			last_uart_log_time=os.time()
			log.warn(tname,"no uart frames="..radar_frame_count.." bytes="..uart_byte_count)
		end
	end

	-- ============== 主循环 ==============
	sys.wait(2000)
	send_dreg()

	while true do
		uart_read_and_parse()
		maintain_tracks(os.time())

		local netr=PronetGetRecChAndDel(nid)
		if netr then
			local obj=json.decode(netr)
			if obj then
				if obj.cmd=="sset" then
					local b={cmd="ssetbck",did=tostring(obj.did or ""),rst=0,times=now_ms()}
					PronetSetSendCh(nid,json.encode(b))
				elseif obj.cmd=="sget" then
					local p=get_datapoint_list(obj.param)
					local b={cmd="sgetbck",did=tostring(obj.did or ""),rst=0,times=now_ms(),param={}}
					fill_param(b.param,p,false)
					PronetSetSendCh(nid,json.encode(b))
				elseif obj.cmd=="dupbck" then
					lasttime=os.time()
				end
			end
		end

		local state_changed=(human_present~=last_human or motion~=last_motion or body_move~=last_body_move or has_person~=last_has_person or people_count~=last_people_count or target_count~=last_target_count or realtime_people_count~=last_realtime_people_count or accurate_people_count~=last_accurate_people_count or primary_target_id~=last_primary_target_id or radar_frame_count~=last_radar_frame_count or traj_x~=last_traj_x or traj_y~=last_traj_y or traj_height~=last_traj_height or traj_speed~=last_traj_speed or uart_bytes~=last_uart_bytes)
		if state_changed then
			last_human=human_present
			last_motion=motion
			last_body_move=body_move
			last_has_person=has_person
			last_people_count=people_count
			last_target_count=target_count
			last_realtime_people_count=realtime_people_count
			last_accurate_people_count=accurate_people_count
			last_primary_target_id=primary_target_id
			last_radar_frame_count=radar_frame_count
			last_traj_x=traj_x
			last_traj_y=traj_y
			last_traj_height=traj_height
			last_traj_speed=traj_speed
			last_uart_bytes=uart_bytes
			needdup=1
		end

		if os.time()-lasttime>180 then needdup=1 end
		if needdup==1 and (force_initial_dup==1 or radar_ready==1 or os.time()-boot_time>=30) then
			local b={cmd="dup",did=next_msg_id(),times=now_ms(),param={}}
			fill_param(b.param,{},true)
			PronetSetSendCh(nid,json.encode(b))
			needdup=0
			force_initial_dup=0
			lasttime=os.time()
		end

		sys.wait(100)
	end
end
