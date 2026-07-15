-- Regression test for Radar_Top_Count_1.3 static hold and raw trajectory de-duplication.
-- Run from this directory: lua test_v1_3_static_hold_and_traj_dedupe.lua

local unpack_fn=table.unpack or unpack

local function read_all(path)
	local f=assert(io.open(path,"rb"))
	local s=f:read("*a")
	f:close()
	return s
end

local function u16(value)
	return math.floor(value/256)%256,value%256
end

local function signed16(value)
	if value<0 then value=0x8000+math.abs(value) end
	return u16(value)
end

local function record(id,x,y,height,speed)
	local xh,xl=signed16(x)
	local yh,yl=signed16(y)
	local hh,hl=u16(height)
	local sh,sl=signed16(speed)
	return {id,1,2,xh,xl,yh,yl,hh,hl,sh,sl}
end

local function flatten(records)
	local data={}
	for i=1,#records do
		for j=1,#records[i] do data[#data+1]=records[i][j] end
	end
	return data
end

local function frame(ctrl,cmd,data)
	local bytes={0x53,0x59,ctrl,cmd,math.floor(#data/256),#data%256}
	for i=1,#data do bytes[#bytes+1]=data[i] end
	local sum=0
	for i=1,#bytes do sum=(sum+bytes[i])%256 end
	bytes[#bytes+1]=sum
	bytes[#bytes+1]=0x54
	bytes[#bytes+1]=0x43
	return string.char(unpack_fn(bytes))
end

local function track_frame(records)
	return frame(0x82,0x02,flatten(records))
end

local function empty_track_frame()
	return frame(0x82,0x02,{})
end

local function run_task(frames,max_waits,start_time)
	local source=read_all("radar_iot_task.lua")
	source=source:gsub("function%s*\n","function __dtu_task()\n",1)
	assert(load(source,"@radar_iot_task.lua","t",_G))()
	assert(type(__dtu_task)=="function","DTU wrapper normalization failed")

	local fake_time=start_time or 10000
	local index=1
	local waits=0
	local dups={}

	os.time=function() return fake_time end
	mobile={
		csq=function() return 20 end,
		imei=function() return "test-imei" end,
		iccid=function() return "test-iccid" end,
		imsi=function() return "test-imsi" end
	}
	log={info=function() end,warn=function() end,error=function() end}
	json={encode=function(v) return v end,decode=function() return nil end}
	sys={wait=function(ms)
		if ms>=1000 then fake_time=fake_time+math.floor(ms/1000) else fake_time=fake_time+1 end
		waits=waits+1
		if waits>max_waits then error("__TEST_STOP__") end
	end}

	function UartStopProRecCh() end
	function PronetStopProRecCh() end
	function PronetGetRecChAndDel() return nil end
	function UartGetRecChAndDel()
		local value=frames[index]
		index=index+1
		return value
	end
	function PronetSetSendCh(_,payload)
		if type(payload)=="table" and payload.cmd=="dup" then dups[#dups+1]=payload end
	end

	local ok,err=pcall(__dtu_task)
	assert(not ok and tostring(err):find("__TEST_STOP__",1,true),"task did not stop through test harness")
	return dups,fake_time
end

local function last_param(dups)
	assert(#dups>0,"missing dup reports")
	return dups[#dups].param
end

local function moving_person(id,base_x,base_y,hit)
	local step=math.min(hit-1,5)*15
	return record(id,base_x+step,base_y,150,20)
end

local function fast_moving_person(id,base_x,base_y,hit)
	local step=math.min(hit-1,3)*30
	return record(id,base_x+step,base_y,150,25)
end

local frames={}
local max_people=7
local hit_counts={0,0,0,0,0,0,0}
local base_x={-150,-150,0,80,80,0,-150}
local base_y={-160,20,-80,60,180,180,140}
for count=1,max_people do
	for hit=1,13 do
		local records={}
		for i=1,count do
			hit_counts[i]=hit_counts[i]+1
			records[#records+1]=moving_person(i,base_x[i],base_y[i],hit_counts[i])
		end
		frames[#frames+1]=track_frame(records)
	end
end
-- Keep all seven tracks visible long enough that the last entrant also meets
-- both mature thresholds before simulating a period of empty track frames.
for _=1,30 do
	local records={}
	for i=1,max_people do
		hit_counts[i]=hit_counts[i]+1
		records[#records+1]=moving_person(i,base_x[i],base_y[i],hit_counts[i])
	end
	frames[#frames+1]=track_frame(records)
end
frames[#frames+1]=frame(0x80,0x01,{0})
frames[#frames+1]=frame(0x86,0x0A,{0,0})
frames[#frames+1]=frame(0x86,0x0C,{0,0})
for _=1,35 do frames[#frames+1]=empty_track_frame() end

local dups=run_task(frames,#frames+2,10000)
local seen_counts={}
for i=1,#dups do seen_counts[dups[i].param.people_count]=true end
for count=1,max_people do assert(seen_counts[count],"missing people_count="..count) end
local p=last_param(dups)
assert(p.pver=="Radar_Top_Count_1.3")
assert(p.people_count==max_people and p.track_people_count==max_people,
	string.format("seven mature static tracks must not clear after 35 seconds (people=%s track=%s)",
		tostring(p.people_count),tostring(p.track_people_count)))
assert(p.has_person==1,"business presence should stay true while mature tracks are held")

local weak_frames={}
-- Eight hits confirms the track but leaves it below the mature hit threshold.
-- Additional non-clear status frames age it past 20 seconds, covering the
-- intermediate state that must still use weak-track cleanup.
for hit=1,8 do weak_frames[#weak_frames+1]=track_frame({fast_moving_person(7,0,0,hit)}) end
for _=1,13 do weak_frames[#weak_frames+1]=frame(0x80,0x01,{1}) end
weak_frames[#weak_frames+1]=frame(0x80,0x01,{0})
weak_frames[#weak_frames+1]=frame(0x86,0x0A,{0,0})
weak_frames[#weak_frames+1]=frame(0x86,0x0C,{0,0})
for _=1,22 do weak_frames[#weak_frames+1]=false end
dups=run_task(weak_frames,#weak_frames+2,20000)
p=last_param(dups)
assert(p.people_count==0 and p.track_people_count==0,"weak confirmed track should clear after all-clear window")
assert(p.target_ids=="","weak cleared track IDs should be empty")

local stale_frames={}
for hit=1,13 do stale_frames[#stale_frames+1]=track_frame({moving_person(8,0,0,hit)}) end
for _=1,31 do stale_frames[#stale_frames+1]=false end
dups=run_task(stale_frames,#stale_frames+2,30000)
p=last_param(dups)
assert(p.radar_ready==0 and p.people_count==0 and p.track_people_count==0,"radar stale must still clear after 30 seconds")

local dedupe_frames={}
for _=1,36 do dedupe_frames[#dedupe_frames+1]=track_frame({record(9,10,20,150,0)}) end
dups=run_task(dedupe_frames,#dedupe_frames+2,40000)
local traj_reports={}
for i=1,#dups do
	if dups[i].param.target_count==1 then traj_reports[#traj_reports+1]=dups[i].param end
end
assert(#traj_reports==3,"unchanged raw track should report first, fifth stable, and 30-second keepalive frames")
assert(traj_reports[1].traj_changed==1 and traj_reports[1].traj_repeat_count==1)
assert(traj_reports[2].traj_stable==1 and traj_reports[2].traj_repeat_count==5)
assert(traj_reports[3].traj_stable==1 and traj_reports[3].traj_repeat_count==35)

dedupe_frames={
	track_frame({record(10,10,20,150,0)}),
	track_frame({record(10,10,20,150,0)}),
	track_frame({record(10,40,20,150,0)})
}
dups=run_task(dedupe_frames,#dedupe_frames+2,50000)
local changed_reports=0
for i=1,#dups do
	if dups[i].param.traj_changed==1 then changed_reports=changed_reports+1 end
end
assert(changed_reports>=2,"raw track changes must trigger immediate reports")

dups=run_task({track_frame({record(11,0,0,150,0)}),empty_track_frame()},5,60000)
p=last_param(dups)
assert(p.target_count==0 and p.traj_changed==1,"empty track frame should report raw track clearance")

print("PASS: V1.3 static hold, weak cleanup, stale clear, and trajectory de-dupe")
