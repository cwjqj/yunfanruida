--[[
============================================================
  R60BMP1 radar people-count core task

  Purpose: keep only the core path needed for cloud upload and
  accurate people_count debugging.

  Count rule:
  - Raw target_count is debug only, never people_count.
  - 0x86 realtime/accurate counts are debug only, never people_count.
  - people_count comes only from confirmed internal tracks.
  - A track is confirmed only after enough hits plus real movement.
  - Static targets are not counted as people.
  - Confirmed tracks are held through static radar loss, then removed
    by exit/lost/stale/fallback rules.
  - uart_bytes reports latest raw UART bytes as HEX text.
============================================================
]]

function
	local tname="RADAR_COUNT_CORE"
	local nid=1
	local pver,fver="Radar_Count_Core_1.0","FW_1.0.0"

	local AREA_X_MIN=-165
	local AREA_X_MAX=165
	local AREA_Y_MIN=0
	local AREA_Y_MAX=500
	local EXIT_ZONE_X_MIN=-120
	local EXIT_ZONE_X_MAX=-20
	local EXIT_ZONE_Y_MIN=0
	local EXIT_ZONE_Y_MAX=70
	local LOWER_AREA_Y_MAX=300

	local TARGET_HEIGHT_MIN_CM=20
	local TARGET_HEIGHT_MAX_CM=245
	local TRACK_TABLE_MAX=32

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
	local ACCURATE_FRESH_SECONDS=90
	local RADAR_STALE_SECONDS=30

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

	local radar_ready=0
	local radar_frame_count=0
	local uart_bytes=""
	local uart_byte_count=0
	local rx_buf=""
	local uid_seq=0
	local tracks={}
	local all_clear_since=0
	local radar_stale_cleared=0

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
	local last_traj_x=-999999
	local last_traj_y=-999999
	local last_traj_height=-999999
	local last_traj_speed=-999999
	local last_uart_bytes=""

	UartStopProRecCh(1)
	PronetStopProRecCh(nid)
	log.info(tname,"start")

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

	local function dist_xy(ax,ay,bx,by)
		local dx=ax-bx
		local dy=ay-by
		return math.sqrt(dx*dx+dy*dy)
	end

	local function in_area(x,y)
		return x>=AREA_X_MIN and x<=AREA_X_MAX and y>=AREA_Y_MIN and y<=AREA_Y_MAX
	end

	local function in_exit_zone(x,y)
		return x>=EXIT_ZONE_X_MIN and x<=EXIT_ZONE_X_MAX and y>=EXIT_ZONE_Y_MIN and y<=EXIT_ZONE_Y_MAX
	end

	local function in_lower_area(x,y)
		return x>=AREA_X_MIN and x<=AREA_X_MAX and y>=AREA_Y_MIN and y<LOWER_AREA_Y_MAX
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

	local function clear_report_targets()
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
			if t.confirmed and (t.state=="active" or t.state=="holding") then n=n+1 end
		end
		people_count=normalize_people_count(n)
		people_min=people_count
		people_max=people_count
		if people_count>0 or human_present==1 then has_person=1 else has_person=0 end
	end

	local function refresh_track_report()
		clear_report_targets()
		local best=nil
		for i=1,#tracks do
			local t=tracks[i]
			if t.confirmed and (t.state=="active" or t.state=="holding") then
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
		people_count=0
		people_min=0
		people_max=0
		has_person=0
		clear_report_targets()
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
		t.in_exit=in_exit_zone(t.x,t.y)
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
			state="candidate",confirmed=false,matched=true,in_exit=in_exit_zone(d.x,d.y)
		}
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
		local accurate_ok=(accurate_people_max==0) or (accurate_updated_at>0 and now-accurate_updated_at>ACCURATE_FRESH_SECONDS)
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

	local function maintain_tracks(now)
		if last_track_frame_time>0 and now-last_track_frame_time>=ACTIVE_TO_HOLD_SEC then
			target_count=0
		end
		for i=#tracks,1,-1 do
			local t=tracks[i]
			local gone=now-t.last_seen
			if not t.confirmed and gone>=CAND_MISS_TTL_SEC then
				remove_track(i,"candidate_timeout")
			elseif t.confirmed and t.state=="active" and gone>=ACTIVE_TO_HOLD_SEC then
				t.state="holding"
			elseif t.confirmed and t.state=="holding" then
				if t.in_exit and gone>=EXIT_GRACE_SEC then
					remove_track(i,"exit_zone")
				elseif in_lower_area(t.last_x or t.x,t.last_y or t.y) and t.last_moving_seen>=t.last_seen-1 and gone>=LOWER_MOVING_LOST_EXIT_SEC then
					remove_track(i,"lower_moving_lost")
				end
			end
		end
		recompute_people_count()
		update_fallback_zero(now)
		if now-last_valid_frame_time>=RADAR_STALE_SECONDS then
			radar_ready=0
			if radar_stale_cleared==0 then
				clear_all("radar_stale")
				radar_stale_cleared=1
			end
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
		if force_all or param_wanted(names,"realtime_people_min") then dst.realtime_people_min=realtime_people_min end
		if force_all or param_wanted(names,"realtime_people_max") then dst.realtime_people_max=realtime_people_max end
		if force_all or param_wanted(names,"realtime_people_count") then dst.realtime_people_count=realtime_people_count end
		if force_all or param_wanted(names,"accurate_people_min") then dst.accurate_people_min=accurate_people_min end
		if force_all or param_wanted(names,"accurate_people_max") then dst.accurate_people_max=accurate_people_max end
		if force_all or param_wanted(names,"accurate_people_count") then dst.accurate_people_count=accurate_people_count end
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
		last_track_frame_time=now
		if #data_bytes==0 then
			target_count=0
			on_track_frame(detections,now)
			return
		end
		if (#data_bytes%11)~=0 then
			target_count=0
			log.warn(tname,"bad track len="..#data_bytes)
			return
		end
		target_count=math.floor(#data_bytes/11)
		for n=0,target_count-1 do
			local o=n*11+1
			local rid=data_bytes[o]
			local x=parse_radar_signed16_be(data_bytes[o+3],data_bytes[o+4])
			local y=parse_radar_signed16_be(data_bytes[o+5],data_bytes[o+6])
			local h=parse_uint16_be(data_bytes[o+7],data_bytes[o+8])
			local s=parse_radar_signed16_be(data_bytes[o+9],data_bytes[o+10])
			if is_valid_height(h) then
				table.insert(detections,{rid=rid,x=x,y=y,height=h,speed=s,matched=false})
			end
		end
		on_track_frame(detections,now)
	end

	local function try_parse_frame()
		local head_pos=string.find(rx_buf,FRAME_HEAD,1,true)
		if not head_pos then rx_buf=""; return false end
		if head_pos>1 then rx_buf=string.sub(rx_buf,head_pos) end
		if string.len(rx_buf)<7 then return false end
		local data_len=string.byte(rx_buf,5)*256+string.byte(rx_buf,6)
		if data_len>256 then rx_buf=string.sub(rx_buf,2); return true end
		local total_len=2+1+1+2+data_len+1+2
		if string.len(rx_buf)<total_len then return false end
		local frame=string.sub(rx_buf,1,total_len)
		rx_buf=string.sub(rx_buf,total_len+1)
		local calc_sum=0
		for i=1,6+data_len do calc_sum=(calc_sum+string.byte(frame,i))%256 end
		local recv_sum=string.byte(frame,6+data_len+1)
		if calc_sum~=recv_sum then log.warn(tname,"checksum fail"); return true end
		if string.byte(frame,6+data_len+2)~=0x54 or string.byte(frame,6+data_len+3)~=0x43 then log.warn(tname,"tail fail"); return true end

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
				realtime_people_min=normalize_people_count(data_bytes[1])
				realtime_people_max=normalize_people_count(data_bytes[2])
				realtime_people_count=realtime_people_max
			elseif cmd==0x0C and #data_bytes>=2 then
				accurate_people_min=normalize_people_count(data_bytes[1])
				accurate_people_max=normalize_people_count(data_bytes[2])
				accurate_people_count=accurate_people_max
				accurate_updated_at=os.time()
			end
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
			log.warn(tname,"no uart frames="..radar_frame_count.." bytes="..uart_byte_count)
		end
	end

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

		local state_changed=(human_present~=last_human or motion~=last_motion or body_move~=last_body_move or has_person~=last_has_person or people_count~=last_people_count or target_count~=last_target_count or realtime_people_count~=last_realtime_people_count or accurate_people_count~=last_accurate_people_count or primary_target_id~=last_primary_target_id or traj_x~=last_traj_x or traj_y~=last_traj_y or traj_height~=last_traj_height or traj_speed~=last_traj_speed or uart_bytes~=last_uart_bytes)
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
