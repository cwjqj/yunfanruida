-- Regression test for Radar_Top_Count_1.3 ROI track counting.
-- Run from this directory: lua test_track_roi_count.lua

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
	return string.char(table.unpack(bytes))
end

local positions={
	{0,210,-210,300},
	{60,270,-270,360},
	{130,340,-340,430},
	{160,410,-410,500}
}

local frames={frame(0x86,0x0C,{2,2})} -- whole-beam accurate count is contaminated
for i=1,#positions do
	local p=positions[i]
	frames[#frames+1]=frame(0x82,0x02,flatten({
		record(1,p[1],0,150,20),    -- intended person inside X/Y range
		record(2,p[2],0,150,20),    -- same-side edge/outside interference
		record(3,p[3],0,150,20),
		record(4,0,p[4],150,20)
	}))
end

local source=read_all("radar_iot_task.lua")
source=source:gsub("function%s*\n","function __dtu_task()\n",1)
assert(load(source,"@radar_iot_task.lua","t",_G))()

local fake_time=3000
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
	if waits>#frames+2 then error("__TEST_STOP__") end
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
assert(not ok and tostring(err):find("__TEST_STOP__",1,true))
assert(#dups>=2)

local p=nil
for i=1,#dups do
	local candidate=dups[i].param
	if candidate.target_count==4 and candidate.track_people_count==1 then
		p=candidate
	end
end
assert(p,"missing report with four raw targets and one confirmed ROI track")
assert(p.pver=="Radar_Top_Count_1.3")
assert(p.people_count_source=="track")
assert(p.accurate_people_count==2,"whole-beam accurate count should remain diagnostic")
assert(p.target_count==4,"raw targets must remain visible")
assert(p.track_people_count==1,"only the confirmed in-range track should count")
assert(p.people_count==1 and p.people_count_valid==1 and p.people_count_pending==0)
assert(p.target_ids=="1","reported stable IDs must use the same ROI filter as people_count")
assert(p.primary_target_id==1)

print("PASS: 1 in-range person counted while 3 confirmed outside/edge targets stay diagnostic")
