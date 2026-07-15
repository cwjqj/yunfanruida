--[[
============================================================
  Yunfan R60BMP1 radar -> YED IOT DTU task

  Track-hold version for dorm people counting.
  - Radar 0x82 trajectory frames are the only source of people_count.
  - 0x86 realtime/accurate people frames are stored and reported only.
  - Confirmed tracks stay counted when the radar loses a static person.
  - A person is removed only after the confirmed track disappears in EXIT_ZONE,
    or after the whole room is proven empty by fallback signals.

  Site calibration checkpoints:
  1. Radar internal install angle must match the physical mount.
  2. Radar internal install height is expected to be 220cm.
  3. Dorm detection range is X=[-165,165]cm, Y=[0,500]cm.
  4. Door/exit zone must be calibrated from the last trajectory point when a
     person leaves. Current default: X=[-120,-20], Y=[0,70].
============================================================
]]

function
	local tname="RADAR_IOT_TRACK"
	log.info(tname,"=== Radar IOT Track Task Start ===")

	local nid=1
	local pver,fver="Radar_DTU_Track_1.0","FW_1.0.0"

	-- Dorm geometry and track parameters, all distances in cm.
	local AREA_X_MIN=-165
	local AREA_X_MAX=165
	local AREA_Y_MIN=0
	local AREA_Y_MAX=500
	local EXIT_ZONE_X_MIN=-120
	local EXIT_ZONE_X_MAX=-20
	local EXIT_ZONE_Y_MIN=0
	local EXIT_ZONE_Y_MAX=70
	local STRICT_OUT_OF_ZONE_EXIT=true

	local ID_MATCH_MAX_JUMP_CM=150
	local NEW_MATCH_DIST_CM=90
	local SPLIT_MERGE_CM=40
	local CONFIRM_HITS=4
	local CONFIRM_MOVE_CM=60
	local STATIC_CONFIRM_ENABLE=false
	local STATIC_CONFIRM_HITS=12
	local STATIC_CONFIRM_STABLE_CM=30
	local LOWER_AREA_Y_MAX=300
	local LOWER_AREA_SPLIT_MERGE_CM=110
	local POS_ALPHA=0.5
	local CAND_MISS_TTL_SEC=2
	local ACTIVE_TO_HOLD_SEC=2
	local EXIT_GRACE_SEC=3
	local LOWER_MOVING_LOST_EXIT_SEC=8
	local MOVING_SPEED_CM_S=10
	local MOVING_STEP_CM=15
	local HOLD_MAX_SEC=0
	local FALLBACK_ZERO_SECONDS=15
	local ACCURATE_FRESH_SECONDS=90
	local RADAR_STALE_SECONDS=30
	local RADAR_STALE_CLEAR=true
	local PEOPLE_COUNT_MAX=10
	local TRACK_TABLE_MAX=16
	local MERGE_DIST_CM=45
	local MERGE_CONFIRM_FRAMES=5
	local TARGET_HEIGHT_MIN_CM=20
	local TARGET_HEIGHT_MAX_CM=245

	-- Posture/area parameters are auxiliary and never change people_count.
	local BED_HEIGHT_CM=212
	local BED_ON_TOLERANCE_CM=25
	local BED_SIT_HEIGHT_CM=220
	local SIDE_AREA_X_MIN_ABS=35
	local BED_AREA_X_MIN_ABS=60
	local SIDE_AREA_X_MAX_ABS=300
	local BED_Y_MIN=0
	local BED_Y_MAX=500
	local UNDER_BED_Y_MIN=80
	local BED_SURFACE_Y_MIN=300
	local STAND_HEIGHT_CM=130
	local SIT_HEIGHT_CM=70
	local POSTURE_HYSTERESIS_CM=10
	local POSTURE_CONFIRM_FRAMES=2

	local human_present=0
	local motion=0
	local body_move=0
	local people_min=0
	local people_max=0
	local realtime_people_min=0
	local realtime_people_max=0
	local realtime_people_count=0
	local accurate_people_min=0
	local accurate_people_max=0
	local accurate_people_count=0
	local accurate_updated_at=0
	local traj_x=0
	local traj_y=0
	local traj_height=0
	local traj_speed=0
	local has_person=0
	local people_count=0
	local under_bed_people_count=0
	local target_count=0
	local frame_target_count=0
	local target_ids=""
	local posture_code=0
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

	local rx_buf=""
	local lasttime=os.time()
	local needdup=1
	local force_initial_dup=1
	local last_dynamic_dup_time=os.time()
	local msg_seq=0
	local boot_time=os.time()
	local radar_ready=0
	local radar_frame_count=0
	local uart_bytes=""
	local uart_byte_count=0
	local last_uart_log_time=os.time()
	local last_valid_frame_time=os.time()
	local all_clear_since=0
	local uid_seq=0
	local person_tracks={}
	local posture_filters={}

	local last_human=-1
	local last_has_person=-1
	local last_motion=-1
	local last_people_min=-1
	local last_people_max=-1
	local last_people_count=-1
	local last_realtime_people_count=-1
	local last_accurate_people_count=-1
	local last_body_move=-1
	local last_traj_x=-999999
	local last_traj_y=-999999
	local last_traj_height=-1
	local last_traj_speed=-999999
	local last_uart_bytes=""
	local last_target_count=-1
	local last_under_bed_people_count=-1
	local last_posture_code=-1

	UartStopProRecCh(1)
	PronetStopProRecCh(nid)
	log.info(tname,"UART1 ready")

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

	local function clamp_people_count(v)
		if not v or v<0 then return 0 end
		if v>PEOPLE_COUNT_MAX then return PEOPLE_COUNT_MAX end
		return v
	end

	local function clamp_value(v,min_value,max_value)
		if v<min_value then return min_value end
		if v>max_value then return max_value end
		return v
	end

	local function dist_xy(ax,ay,bx,by)
		local dx=ax-bx
		local dy=ay-by
		return math.sqrt(dx*dx+dy*dy)
	end

	local function in_dorm_area(x,y)
		return x>=AREA_X_MIN and x<=AREA_X_MAX and y>=AREA_Y_MIN and y<=AREA_Y_MAX
	end

	local function in_lower_count_area(x,y)
		return x>=AREA_X_MIN and x<=AREA_X_MAX and y>=AREA_Y_MIN and y<LOWER_AREA_Y_MAX
	end

	local function split_merge_distance(ax,ay,bx,by)
		if in_lower_count_area(ax,ay) and in_lower_count_area(bx,by) then
			return LOWER_AREA_SPLIT_MERGE_CM
		end
		return SPLIT_MERGE_CM
	end

	local function in_exit_zone(x,y)
		return x>=EXIT_ZONE_X_MIN and x<=EXIT_ZONE_X_MAX and y>=EXIT_ZONE_Y_MIN and y<=EXIT_ZONE_Y_MAX
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
		return in_side_area(x,y) and y>=UNDER_BED_Y_MIN and y<BED_SURFACE_Y_MIN
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
			if h>=bed_sit_line then return 2,"坐下" end
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

	local function stable_posture_for_target(uid,h,x,y)
		local key=tostring(uid)
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

	local function remove_track(index,reason)
		local track=person_tracks[index]
		if track then
			posture_filters[tostring(track.uid)]=nil
			log.info(tname,string.format("remove track uid=%d reason=%s x=%d y=%d state=%s confirmed=%s",track.uid,tostring(reason),track.x,track.y,tostring(track.state),tostring(track.confirmed)))
		end
		table.remove(person_tracks,index)
	end

	local function report_x(track)
		if track.last_dorm_x then return track.last_dorm_x end
		return clamp_value(track.x,AREA_X_MIN,AREA_X_MAX)
	end

	local function report_y(track)
		if track.last_dorm_y then return track.last_dorm_y end
		return clamp_value(track.y,AREA_Y_MIN,AREA_Y_MAX)
	end

	local function report_height(track)
		return track.last_dorm_height or track.height
	end

	local function report_speed(track)
		return track.last_dorm_speed or track.speed
	end

	local function recompute_people_count()
		local n=0
		for i=1,#person_tracks do
			local track=person_tracks[i]
			if track.confirmed and (track.state=="active" or track.state=="holding") then
				n=n+1
			end
		end
		people_count=clamp_people_count(n)
		people_min=people_count
		people_max=people_count
		if people_count>0 or human_present==1 then has_person=1 else has_person=0 end
	end

	local function try_confirm(track,now)
		if track.confirmed then return end
		if track.hits>=CONFIRM_HITS and track.max_disp>=CONFIRM_MOVE_CM then
			track.confirmed=true
			log.info(tname,string.format("track confirmed uid=%d mode=move hits=%d disp=%d",track.uid,track.hits,track.max_disp))
			return
		end
		if STATIC_CONFIRM_ENABLE and track.hits>=STATIC_CONFIRM_HITS and human_present==1 and track.max_disp<=STATIC_CONFIRM_STABLE_CM then
			track.confirmed=true
			log.info(tname,string.format("track confirmed uid=%d mode=static hits=%d disp=%d",track.uid,track.hits,track.max_disp))
		end
	end

	local function bind_detection(track,detection,now)
		detection.matched=true
		track.matched_this_frame=true
		local old_x=track.x
		local old_y=track.y
		track.x=math.floor(track.x*(1-POS_ALPHA)+detection.x*POS_ALPHA+0.5)
		track.y=math.floor(track.y*(1-POS_ALPHA)+detection.y*POS_ALPHA+0.5)
		track.height=detection.height
		track.speed=detection.speed
		track.rid=detection.rid
		track.hits=track.hits+1
		track.last_seen=now
		track.in_exit=in_exit_zone(track.x,track.y)
		track.outside_area=not in_dorm_area(track.x,track.y)
		local step=dist_xy(old_x,old_y,track.x,track.y)
		if math.abs(track.speed)>=MOVING_SPEED_CM_S or step>=MOVING_STEP_CM then
			track.last_moving_seen=now
		end
		if in_dorm_area(track.x,track.y) then
			track.last_dorm_x=track.x
			track.last_dorm_y=track.y
			track.last_dorm_height=track.height
			track.last_dorm_speed=track.speed
		end
		local disp=dist_xy(track.x,track.y,track.birth_x,track.birth_y)
		if disp>track.max_disp then track.max_disp=math.floor(disp+0.5) end
		try_confirm(track,now)
		if track.confirmed then track.state="active" end
	end

	local function create_track(detection,now)
		if #person_tracks>=TRACK_TABLE_MAX then
			log.warn(tname,"track table full, drop new detection")
			return
		end
		for i=1,#person_tracks do
			local track=person_tracks[i]
			local merge_dist=split_merge_distance(track.x,track.y,detection.x,detection.y)
			if dist_xy(track.x,track.y,detection.x,detection.y)<merge_dist and (track.confirmed or in_lower_count_area(track.x,track.y)) then
				return
			end
		end
		uid_seq=uid_seq+1
		local track={
			uid=uid_seq,
			rid=detection.rid,
			x=detection.x,
			y=detection.y,
			height=detection.height,
			speed=detection.speed,
			birth_x=detection.x,
			birth_y=detection.y,
			max_disp=0,
			hits=1,
			first_seen=now,
			last_seen=now,
			state="candidate",
			confirmed=false,
			in_exit=in_exit_zone(detection.x,detection.y),
			outside_area=not in_dorm_area(detection.x,detection.y),
			last_moving_seen=(math.abs(detection.speed)>=MOVING_SPEED_CM_S) and now or 0,
			close_hits=0,
			matched_this_frame=true
		}
		if in_dorm_area(detection.x,detection.y) then
			track.last_dorm_x=detection.x
			track.last_dorm_y=detection.y
			track.last_dorm_height=detection.height
			track.last_dorm_speed=detection.speed
		end
		table.insert(person_tracks,track)
		detection.matched=true
		log.info(tname,string.format("new track uid=%d rid=%d x=%d y=%d h=%d",track.uid,track.rid,track.x,track.y,track.height))
	end

	local function merge_close_tracks(now)
		for i=#person_tracks,1,-1 do
			local a=person_tracks[i]
			for j=i-1,1,-1 do
				local b=person_tracks[j]
				local merge_dist=MERGE_DIST_CM
				if in_lower_count_area(a.x,a.y) and in_lower_count_area(b.x,b.y) then
					merge_dist=LOWER_AREA_SPLIT_MERGE_CM
				end
				if dist_xy(a.x,a.y,b.x,b.y)<merge_dist then
					a.close_hits=(a.close_hits or 0)+1
					b.close_hits=(b.close_hits or 0)+1
					if a.close_hits>=MERGE_CONFIRM_FRAMES or b.close_hits>=MERGE_CONFIRM_FRAMES then
						local keep_index=j
						local drop_index=i
						if a.hits>b.hits then
							keep_index=i
							drop_index=j
						end
						local keep=person_tracks[keep_index]
						local drop=person_tracks[drop_index]
						keep.confirmed=keep.confirmed or drop.confirmed
						if keep.state=="candidate" and keep.confirmed then keep.state="active" end
						if drop.last_dorm_x and not keep.last_dorm_x then
							keep.last_dorm_x=drop.last_dorm_x
							keep.last_dorm_y=drop.last_dorm_y
							keep.last_dorm_height=drop.last_dorm_height
							keep.last_dorm_speed=drop.last_dorm_speed
						end
						remove_track(drop_index,"merge_close")
						return
					end
				else
					a.close_hits=0
					b.close_hits=0
				end
			end
		end
	end

	local function on_track_frame(detections,now)
		last_valid_frame_time=now
		for i=1,#person_tracks do
			person_tracks[i].matched_this_frame=false
		end

		for i=1,#detections do
			local detection=detections[i]
			local best=nil
			local best_dist=ID_MATCH_MAX_JUMP_CM+1
			for j=1,#person_tracks do
				local track=person_tracks[j]
				if not track.matched_this_frame and track.rid==detection.rid then
					local dd=dist_xy(track.x,track.y,detection.x,detection.y)
					if dd<best_dist then
						best=track
						best_dist=dd
					end
				end
			end
			if best and best_dist<=ID_MATCH_MAX_JUMP_CM then
				bind_detection(best,detection,now)
			end
		end

		local pairs={}
		for i=1,#detections do
			local detection=detections[i]
			if not detection.matched then
				for j=1,#person_tracks do
					local track=person_tracks[j]
					if not track.matched_this_frame then
						local dd=dist_xy(track.x,track.y,detection.x,detection.y)
						if dd<=NEW_MATCH_DIST_CM then
							table.insert(pairs,{detection=detection,track=track,dist=dd})
						end
					end
				end
			end
		end
		table.sort(pairs,function(a,b) return a.dist<b.dist end)
		for i=1,#pairs do
			local pair=pairs[i]
			if not pair.detection.matched and not pair.track.matched_this_frame then
				bind_detection(pair.track,pair.detection,now)
			end
		end

		for i=1,#detections do
			local detection=detections[i]
			if not detection.matched then
				create_track(detection,now)
			end
		end
		merge_close_tracks(now)
	end

	local function refresh_target_report()
		clear_targets()
		local report_tracks={}
		for i=1,#person_tracks do
			local track=person_tracks[i]
			if track.confirmed and (track.state=="active" or track.state=="holding") then
				table.insert(report_tracks,track)
			end
		end
		table.sort(report_tracks,function(a,b)
			if a.state~=b.state then return a.state=="active" end
			if a.hits~=b.hits then return a.hits>b.hits end
			return a.uid<b.uid
		end)
		target_count=#report_tracks
		for i=1,#report_tracks do
			local track=report_tracks[i]
			local x=report_x(track)
			local y=report_y(track)
			local h=report_height(track)
			local s=report_speed(track)
			local pcode,ptext=stable_posture_for_target(track.uid,h,x,y)
			if i==1 then target_ids=tostring(track.uid) else target_ids=target_ids..","..tostring(track.uid) end
			if i<=3 then set_target_slot(i-1,track.uid,x,y,h,s,pcode,ptext) end
			if i==1 then
				traj_x=x
				traj_y=y
				traj_height=h
				traj_speed=s
				primary_target_id=track.uid
				primary_target_x=x
				primary_target_y=y
				primary_target_height=h
				primary_target_speed=s
				primary_target_posture=pcode
				primary_in_bed_zone=in_bed_zone(x,y) and 1 or 0
				posture_code=pcode
				posture_text=ptext
				main_posture_text=ptext
			end
		end
	end

	local function clear_all_tracks(reason)
		person_tracks={}
		posture_filters={}
		people_count=0
		people_min=0
		people_max=0
		has_person=0
		clear_targets()
		log.info(tname,"clear all tracks reason="..tostring(reason))
	end

	local function update_fallback_zero(now)
		local accurate_ok=(accurate_people_max==0) or (accurate_updated_at>0 and now-accurate_updated_at>ACCURATE_FRESH_SECONDS)
		local all_clear=(human_present==0 and realtime_people_max==0 and accurate_ok)
		if all_clear then
			if all_clear_since==0 then all_clear_since=now end
			if now-all_clear_since>=FALLBACK_ZERO_SECONDS then
				clear_all_tracks("fallback_zero")
				all_clear_since=now
			end
		else
			all_clear_since=0
		end
	end

	local function maintain_tracks(now)
		for i=#person_tracks,1,-1 do
			local track=person_tracks[i]
			local gone=now-track.last_seen
			if not track.confirmed and gone>=CAND_MISS_TTL_SEC then
				remove_track(i,"candidate_timeout")
			elseif track.confirmed and track.state=="active" and gone>=ACTIVE_TO_HOLD_SEC then
				track.state="holding"
				log.info(tname,string.format("track holding uid=%d x=%d y=%d in_exit=%s",track.uid,report_x(track),report_y(track),tostring(track.in_exit)))
			elseif track.confirmed and track.state=="holding" then
				if STRICT_OUT_OF_ZONE_EXIT and track.outside_area then
					remove_track(i,"outside_area")
				elseif track.in_exit and gone>=EXIT_GRACE_SEC then
					remove_track(i,"exit_zone_timeout")
				elseif in_lower_count_area(report_x(track),report_y(track)) and track.last_moving_seen and track.last_moving_seen>=track.last_seen-1 and gone>=LOWER_MOVING_LOST_EXIT_SEC then
					remove_track(i,"lower_moving_lost")
				elseif HOLD_MAX_SEC>0 and gone>=HOLD_MAX_SEC then
					remove_track(i,"hold_max_timeout")
				end
			end
		end
		recompute_people_count()
		update_fallback_zero(now)
		if now-last_valid_frame_time>=RADAR_STALE_SECONDS then
			radar_ready=0
			if RADAR_STALE_CLEAR then clear_all_tracks("radar_stale") end
		end
		refresh_target_report()
	end

	local function accept_realtime_people_count(raw_min,raw_max)
		realtime_people_min=clamp_people_count(raw_min)
		realtime_people_max=clamp_people_count(raw_max)
		realtime_people_count=realtime_people_max
		log.info(tname,string.format("realtime people raw min=%d max=%d report_only",raw_min,raw_max))
	end

	local function accept_accurate_people_count(raw_min,raw_max)
		accurate_people_min=clamp_people_count(raw_min)
		accurate_people_max=clamp_people_count(raw_max)
		accurate_people_count=accurate_people_max
		accurate_updated_at=os.time()
		log.info(tname,string.format("accurate people raw min=%d max=%d report_only",raw_min,raw_max))
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
		if force_all or param_wanted(names,"frame_target_count") then dst.frame_target_count=frame_target_count end
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

	local FRAME_HEAD=string.char(0x53,0x59)

	local function parse_track_payload(data_bytes,now)
		local detections={}
		if #data_bytes==0 then
			frame_target_count=0
			on_track_frame(detections,now)
			return
		end
		if (#data_bytes%11)~=0 then
			log.warn(tname,"track payload length not divisible by 11 len="..#data_bytes)
			return
		end
		frame_target_count=math.floor(#data_bytes/11)
		for n=0,frame_target_count-1 do
			local offset=n*11+1
			local rid=data_bytes[offset]
			local x=parse_radar_signed16_be(data_bytes[offset+3],data_bytes[offset+4])
			local y=parse_radar_signed16_be(data_bytes[offset+5],data_bytes[offset+6])
			local h=parse_uint16_be(data_bytes[offset+7],data_bytes[offset+8])
			local s=parse_radar_signed16_be(data_bytes[offset+9],data_bytes[offset+10])
			if is_valid_target_height(h) then
				table.insert(detections,{rid=rid,x=x,y=y,height=h,speed=s,matched=false})
			else
				log.info(tname,string.format("drop detection rid=%d x=%d y=%d h=%d reason=height",rid,x,y,h))
			end
		end
		on_track_frame(detections,now)
		log.info(tname,string.format("track frame raw=%d valid=%d confirmed=%d ids=%s",frame_target_count,#detections,people_count,target_ids))
	end

	local function try_parse_frame()
		local head_pos=string.find(rx_buf,FRAME_HEAD,1,true)
		if not head_pos then
			rx_buf=""
			return false
		end
		if head_pos>1 then rx_buf=string.sub(rx_buf,head_pos) end
		if string.len(rx_buf)<7 then return false end

		local data_len=string.byte(rx_buf,5)*256+string.byte(rx_buf,6)
		if data_len>256 then
			log.warn(tname,"invalid data_len="..data_len)
			rx_buf=string.sub(rx_buf,2)
			return true
		end
		local total_len=2+1+1+2+data_len+1+2
		if string.len(rx_buf)<total_len then return false end

		local frame=string.sub(rx_buf,1,total_len)
		rx_buf=string.sub(rx_buf,total_len+1)
		local control_word=string.byte(frame,3)
		local command_word=string.byte(frame,4)
		local calc_sum=0
		for i=1,6+data_len do
			calc_sum=(calc_sum+string.byte(frame,i))%256
		end
		local recv_sum=string.byte(frame,6+data_len+1)
		if calc_sum~=recv_sum then
			log.warn(tname,"checksum fail calc="..calc_sum.." recv="..recv_sum)
			return true
		end
		local tail1=string.byte(frame,6+data_len+2)
		local tail2=string.byte(frame,6+data_len+3)
		if tail1~=0x54 or tail2~=0x43 then
			log.warn(tname,"tail error")
			return true
		end

		radar_ready=1
		radar_frame_count=radar_frame_count+1
		local data_bytes={}
		for i=7,6+data_len do
			table.insert(data_bytes,string.byte(frame,i))
		end

		if control_word==0x01 then
			log.info(tname,"heartbeat")
		elseif control_word==0x80 then
			if (command_word==0x01 or command_word==0x81) and #data_bytes>=1 then
				human_present=(data_bytes[1]==1) and 1 or 0
				if human_present==1 then has_person=1 end
				log.info(tname,"human raw="..human_present)
			elseif (command_word==0x02 or command_word==0x82) and #data_bytes>=1 then
				motion=data_bytes[1]
				log.info(tname,"motion="..motion)
			elseif (command_word==0x03 or command_word==0x83) and #data_bytes>=1 then
				body_move=data_bytes[1]
				log.info(tname,"body_move="..body_move)
			end
		elseif control_word==0x82 then
			if command_word==0x02 or command_word==0x82 then
				parse_track_payload(data_bytes,os.time())
			end
		elseif control_word==0x86 then
			if command_word==0x0A and #data_bytes>=2 then
				accept_realtime_people_count(data_bytes[1],data_bytes[2])
			elseif command_word==0x0C and #data_bytes>=2 then
				accept_accurate_people_count(data_bytes[1],data_bytes[2])
			end
		else
			log.info(tname,"ctrl=0x"..string.format("%02X",control_word).." cmd="..command_word)
		end
		return true
	end

	local function uart_read_and_parse()
		local data=UartGetRecChAndDel(1)
		if data and string.len(data)>0 then
			uart_byte_count=uart_byte_count+string.len(data)
			uart_bytes=bytes_to_hex(data)
			rx_buf=rx_buf..data
			while try_parse_frame() do end
		elseif os.time()-last_uart_log_time>=30 then
			last_uart_log_time=os.time()
			log.warn(tname,"no radar uart data, frames="..radar_frame_count.." bytes="..uart_byte_count)
		end
	end

	sys.wait(2000)
	send_dreg()
	lasttime=os.time()
	last_human=human_present
	last_has_person=has_person
	last_motion=motion
	last_body_move=body_move
	last_people_min=people_min
	last_people_max=people_max
	last_people_count=people_count
	last_realtime_people_count=realtime_people_count
	last_accurate_people_count=accurate_people_count
	last_traj_x=traj_x
	last_traj_y=traj_y
	last_traj_height=traj_height
	last_traj_speed=traj_speed
	last_uart_bytes=uart_bytes
	last_target_count=target_count
	last_under_bed_people_count=under_bed_people_count
	last_posture_code=posture_code

	while true do
		uart_read_and_parse()
		maintain_tracks(os.time())

		local netr=PronetGetRecChAndDel(nid)
		if netr then
			log.info(tname,"netr: "..netr)
			local obj=json.decode(netr)
			if obj then
				if obj.cmd=="sset" then
					local b={}
					b.cmd="ssetbck"
					b.did=tostring(obj.did or "")
					b.rst=0
					b.times=now_ms()
					local s=json.encode(b)
					log.info(tname,"ssetbck: "..s)
					PronetSetSendCh(nid,s)
				elseif obj.cmd=="sget" then
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

		local now=os.time()
		local state_changed=(human_present~=last_human or has_person~=last_has_person or motion~=last_motion or people_min~=last_people_min or people_max~=last_people_max or people_count~=last_people_count or realtime_people_count~=last_realtime_people_count or accurate_people_count~=last_accurate_people_count or target_count~=last_target_count or under_bed_people_count~=last_under_bed_people_count or posture_code~=last_posture_code)
		local dynamic_changed=(body_move~=last_body_move or traj_x~=last_traj_x or traj_y~=last_traj_y or traj_height~=last_traj_height or traj_speed~=last_traj_speed or uart_bytes~=last_uart_bytes)
		if state_changed or dynamic_changed then
			last_human=human_present
			last_has_person=has_person
			last_motion=motion
			last_body_move=body_move
			last_people_min=people_min
			last_people_max=people_max
			last_people_count=people_count
			last_realtime_people_count=realtime_people_count
			last_accurate_people_count=accurate_people_count
			last_traj_x=traj_x
			last_traj_y=traj_y
			last_traj_height=traj_height
			last_traj_speed=traj_speed
			last_uart_bytes=uart_bytes
			last_target_count=target_count
			last_under_bed_people_count=under_bed_people_count
			last_posture_code=posture_code
			last_dynamic_dup_time=now
			needdup=1
			log.info(tname,"state changed need dup")
		end

		if os.time()-lasttime>180 then
			needdup=1
			log.info(tname,"heartbeat timeout need dup")
		end

		if needdup==1 and (force_initial_dup==1 or radar_ready==1 or os.time()-boot_time>=30) then
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
			force_initial_dup=0
			lasttime=os.time()
		end

		sys.wait(100)
	end
end
