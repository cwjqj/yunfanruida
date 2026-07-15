-- Offline regression tests for radar_iot_task_door_exit.lua.
-- Run from 顶装/脚本/lua: lua tests/door_exit/test_radar_iot_task_door_exit.lua

local function read_all(path)
	local f=assert(io.open(path,"rb"))
	local s=f:read("*a")
	f:close()
	return s
end

local function signed_bytes(value)
	local encoded=value
	if value<0 then encoded=0x8000+math.abs(value) end
	return math.floor(encoded/256)%256,encoded%256
end

local function u16_bytes(value)
	return math.floor(value/256)%256,value%256
end

local function track_record(id,x,y,height,speed)
	local xh,xl=signed_bytes(x)
	local yh,yl=signed_bytes(y)
	local hh,hl=u16_bytes(height)
	local sh,sl=signed_bytes(speed)
	return {id,1,0,xh,xl,yh,yl,hh,hl,sh,sl}
end

local function track_data(targets)
	local data={}
	for i=1,#targets do
		local t=targets[i]
		local record=track_record(t.id,t.x,t.y,t.height or 150,t.speed or 20)
		for j=1,#record do data[#data+1]=record[j] end
	end
	return data
end

local function make_frame(ctrl,cmd,data)
	local len=#data
	local bytes={0x53,0x59,ctrl,cmd,math.floor(len/256)%256,len%256}
	for i=1,len do bytes[#bytes+1]=data[i] end
	local sum=0
	for i=1,#bytes do sum=(sum+bytes[i])%256 end
	bytes[#bytes+1]=sum
	bytes[#bytes+1]=0x54
	bytes[#bytes+1]=0x43
	return string.char(table.unpack(bytes))
end

local function track_frame(targets)
	return make_frame(0x82,0x02,track_data(targets))
end

local function empty_track_frame()
	return make_frame(0x82,0x02,{})
end

local source=read_all("radar_iot_task_door_exit.lua")
source=source:gsub("function%s*\n","function __door_exit_task()\n",1)
assert(load(source,"@radar_iot_task_door_exit.lua","t",_G))()
assert(type(__door_exit_task)=="function","DTU wrapper normalization failed")

local frames={}
local frame_index=1
local fake_time=1000
local wait_count=0
local max_waits=0
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

local function run(sequence,start_time)
	frames=sequence
	frame_index=1
	fake_time=start_time
	wait_count=0
	max_waits=#frames+2
	dups={}
	local ok,err=pcall(__door_exit_task)
	assert(not ok and tostring(err):find("__TEST_STOP__",1,true),"task did not stop through test harness")
	assert(#dups>=#frames,"expected state reports for parsed frames")
	return dups[#dups].param,dups
end

local function one(id,x,y,speed)
	return {{id=id,x=x,y=y,height=150,speed=speed or 20}}
end

local function presence_frame(value)
	return make_frame(0x80,0x01,{value})
end

local function door_event_frame(value)
	return make_frame(0x86,0x14,{value})
end

-- Scenario 1: a confirmed person disappears inside the room. The track must
-- remain holding and people_count must not decrease.
local indoor={presence_frame(1)}
for _,x in ipairs({0,20,60,80,100}) do indoor[#indoor+1]=track_frame(one(0,x,180,20)) end
for _=1,5 do indoor[#indoor+1]=empty_track_frame() end
local p=run(indoor,1000)
assert(p.people_count==1,"indoor loss must keep the confirmed person")
assert(p.track_people_count==1 and p.people_count_source=="track")

-- Scenario 2: the same confirmed track returns to the left-wall door label,
-- obtains two door hits, then disappears. It must be removed individually.
local exiting={presence_frame(1)}
for _,x in ipairs({0,20,60,80,100,40,-20,-70,-115,-150,-170}) do
	exiting[#exiting+1]=track_frame(one(0,x,0,20))
end
for _=1,5 do exiting[#exiting+1]=empty_track_frame() end
p=run(exiting,2000)
assert(p.people_count==0,"door-side loss must remove the exiting track")
assert(p.track_people_count==0 and p.target_ids=="")

-- Scenario 3: five confirmed people are present. Target 2 walks to the door
-- and disappears while targets 1..4 remain visible; the count must become 4.
local five={presence_frame(1)}
local ys={-200,-100,0,100,200}
for _,x in ipairs({0,20,60,80,100}) do
	local targets={}
	for id=0,4 do targets[#targets+1]={id=id,x=x,y=ys[id+1],height=150,speed=20} end
	five[#five+1]=track_frame(targets)
end
for _,x in ipairs({40,-20,-70,-115,-150,-170}) do
	local targets={{id=2,x=x,y=0,height=150,speed=20}}
	for id=0,4 do
		if id~=2 then targets[#targets+1]={id=id,x=100,y=ys[id+1],height=150,speed=0} end
	end
	five[#five+1]=track_frame(targets)
end
for _=1,5 do
	local targets={}
	for id=0,4 do
		if id~=2 then targets[#targets+1]={id=id,x=100,y=ys[id+1],height=150,speed=0} end
	end
	five[#five+1]=track_frame(targets)
end
p=run(five,3000)
assert(p.people_count==4,"five-person scenario expected 4, got "..tostring(p.people_count).." ids="..tostring(p.target_ids))
assert(p.track_people_count==4,"track diagnostic count expected 4, got "..tostring(p.track_people_count))

-- Scenario 4: a confirmed track approaches the door slowly, so its old
-- movement evidence expires. A module door-out event still authorizes removal
-- after the track becomes holding, without directly subtracting a number.
local door_event={presence_frame(1)}
for _,x in ipairs({0,20,60,80,100}) do door_event[#door_event+1]=track_frame(one(0,x,0,20)) end
for x=80,-200,-20 do door_event[#door_event+1]=track_frame(one(0,x,0,0)) end
door_event[#door_event+1]=door_event_frame(1)
for _=1,4 do door_event[#door_event+1]=empty_track_frame() end
p=run(door_event,4000)
assert(p.people_count==0,"door-out event must remove one eligible door-side holding track")
assert(p.door_event==1 and p.door_event_count==1)

-- Scenario 5: six confirmed tracks disagree with the module's stable accurate
-- count of five. Only the stale holding track that reached the door may be
-- pruned; the five visible room tracks must remain.
local capped={presence_frame(1)}
local six_ys={-225,-135,-45,45,135,225}
for _,x in ipairs({0,20,60,80,100}) do
	local targets={}
	for id=0,5 do targets[#targets+1]={id=id,x=x,y=six_ys[id+1],height=150,speed=20} end
	capped[#capped+1]=track_frame(targets)
end
capped[#capped+1]=make_frame(0x86,0x0C,{5,5})
for x=90,-200,-10 do
	local targets={{id=2,x=x,y=-45,height=150,speed=0}}
	for id=0,5 do
		if id~=2 then targets[#targets+1]={id=id,x=100,y=six_ys[id+1],height=150,speed=0} end
	end
	capped[#capped+1]=track_frame(targets)
end
for _=1,8 do
	local targets={}
	for id=0,5 do
		if id~=2 then targets[#targets+1]={id=id,x=100,y=six_ys[id+1],height=150,speed=0} end
	end
	capped[#capped+1]=track_frame(targets)
end
p=run(capped,5000)
assert(p.people_count==5,"configured maximum reconciliation expected 5, got "..tostring(p.people_count))
assert(p.track_people_count==5,"configured maximum must delete the door ghost, not merely cap output")

print("PASS: indoor loss retained, door exit removed, five-person exit became four, door event matched one track, door ghost pruned to max five")
