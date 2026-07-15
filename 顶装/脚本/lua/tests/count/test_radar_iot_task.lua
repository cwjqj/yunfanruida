-- Offline regression test for radar_iot_task.lua.
-- Run from 顶装/脚本/lua: lua tests/count/test_radar_iot_task.lua

local function read_all(path)
	local f=assert(io.open(path,"rb"))
	local s=f:read("*a")
	f:close()
	return s
end

local function make_frame(ctrl,cmd,data)
	local bytes={0x53,0x59,ctrl,cmd,0, #data}
	for i=1,#data do bytes[#bytes+1]=data[i] end
	local sum=0
	for i=1,#bytes do sum=(sum+bytes[i])%256 end
	bytes[#bytes+1]=sum
	bytes[#bytes+1]=0x54
	bytes[#bytes+1]=0x43
	return string.char(table.unpack(bytes))
end

local frames={
	make_frame(0x86,0x0A,{2,2}), -- realtime says 2, still waiting accurate
	make_frame(0x86,0x0C,{1,1}), -- accurate says 1, realtime disagreement stays pending
	make_frame(0x86,0x14,{1}),   -- out event, never decrements directly
	make_frame(0x86,0x0A,{1,1}), -- realtime returns to the accepted count
	make_frame(0x86,0x0C,{1,1}), -- next accurate frame clears pending
	make_frame(0x86,0x0C,{1,2}), -- uncertain range keeps the previous final count
	make_frame(0x86,0x0A,{2,2}), -- preview of a new count
	make_frame(0x86,0x0C,{2,2})  -- accurate confirmation accepts 2
}

for _=1,95 do
	frames[#frames+1]=make_frame(0x01,0x01,{0x0F}) -- keep UART alive while accurate ages
end

local source=read_all("radar_iot_task.lua")
-- The current Count entry uses ROI tracks. This test keeps the accurate-source
-- rollback mode covered by overriding only the configuration literal.
source=source:gsub('source="track",','source="accurate",',1)
source=source:gsub("function%s*\n","function __dtu_task()\n",1)
assert(load(source,"@radar_iot_task.lua","t",_G))()
assert(type(__dtu_task)=="function","DTU wrapper normalization failed")

local fake_time=1000
local frame_index=1
local wait_count=0
local max_waits=#frames+2
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
sys={
	wait=function(ms)
		if ms>=1000 then fake_time=fake_time+math.floor(ms/1000) else fake_time=fake_time+1 end
		wait_count=wait_count+1
		if wait_count>max_waits then error("__TEST_STOP__") end
	end
}

function UartStopProRecCh() end
function PronetStopProRecCh() end
function PronetGetRecChAndDel() return nil end
function UartGetRecChAndDel()
	local frame=frames[frame_index]
	frame_index=frame_index+1
	return frame
end
function PronetSetSendCh(_,payload)
	if type(payload)=="table" and payload.cmd=="dup" then dups[#dups+1]=payload end
end

local ok,err=pcall(__dtu_task)
assert(not ok and tostring(err):find("__TEST_STOP__",1,true),"task did not stop through test harness")
assert(#dups>=8,"expected state reports for meaningful count transitions")

local function param(n)
	assert(dups[n] and dups[n].param,"missing dup #"..tostring(n))
	return dups[n].param
end

local p=param(1)
assert(p.people_count==0 and p.people_count_valid==0 and p.people_count_pending==1)
assert(p.realtime_people_count==2 and p.people_count_source=="accurate")

p=param(2)
assert(p.people_count==1 and p.people_count_valid==1 and p.people_count_pending==1)

p=param(3)
assert(p.people_count==1 and p.door_event==1 and p.door_event_count==1)
assert(p.people_count_pending==1)

p=param(5)
assert(p.people_count==1 and p.people_count_valid==1 and p.people_count_pending==0)

p=param(6)
assert(p.people_count==1 and p.accurate_people_min==1 and p.accurate_people_max==2)
assert(p.people_count_pending==1)

local accepted_two=nil
for i=1,#dups do
	local candidate=dups[i].param
	if candidate.people_count==2 and candidate.people_count_valid==1 and candidate.people_count_pending==0 then
		accepted_two=candidate
		break
	end
end
p=assert(accepted_two,"missing accepted accurate count of 2")
assert(p.people_count==2 and p.people_count_valid==1 and p.people_count_pending==0)

p=dups[#dups].param
assert(p.radar_ready==1,"heartbeats should keep radar ready")
assert(p.people_count==2,"stale accurate data must retain the last confirmed value")
assert(p.people_count_valid==0 and p.people_count_pending==1)
assert(p.has_person==0,"stale invalid count must not keep business presence without human_present")
assert(p.accurate_age_sec>=91,"accurate count should be stale after 90 seconds")

print(string.format("PASS: %d frames, %d dup reports, final accurate age=%d",#frames,#dups,p.accurate_age_sec))

-- A separate task run verifies that a dead UART clears the accepted count.
frames={make_frame(0x86,0x0C,{2,2})}
frame_index=1
wait_count=0
max_waits=40
dups={}
fake_time=2000

ok,err=pcall(__dtu_task)
assert(not ok and tostring(err):find("__TEST_STOP__",1,true),"stale-UART scenario did not stop")
assert(#dups>=2,"stale-UART scenario should report accepted and cleared states")
assert(dups[1].param.people_count==2 and dups[1].param.people_count_valid==1)
p=dups[#dups].param
assert(p.radar_ready==0 and p.people_count==0)
assert(p.people_count_valid==0 and p.people_count_pending==1)
assert(p.accurate_age_sec==-1)

print(string.format("PASS: radar stale scenario cleared count (%d simulated seconds total)",fake_time-2000))
