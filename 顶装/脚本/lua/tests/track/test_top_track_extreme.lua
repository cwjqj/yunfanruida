-- Software stress test for Radar_Top_Track_2.0.
-- The parser accepts at most floor(256/11)=23 complete target records.

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

local bases={}
for _,y in ipairs({-150,-50,50,150}) do
	for _,x in ipairs({-150,-90,-30,30,90,150}) do
		if #bases<23 then bases[#bases+1]={x=x,y=y} end
	end
end

local frames={}
for step=0,3 do
	local data={}
	for i=1,#bases do
		local base=bases[i]
		local direction=(base.x<0) and 1 or -1
		local rec=record(i-1,base.x+direction*step*30,base.y,140+(i%5)*5,20)
		for j=1,#rec do data[#data+1]=rec[j] end
	end
	assert(#data==253,"23 target records must occupy 253 bytes")
	frames[#frames+1]=frame(0x82,0x02,data)
end

local source=read_all("radar_iot_task_track.lua")
source=source:gsub("function%s*\n","function __dtu_task()\n",1)
assert(load(source,"@radar_iot_task_track.lua","t",_G))()

local fake_time=1000
local index=1
local waits=0
local dups={}

os.time=function() return fake_time end
mobile={csq=function() return 20 end,imei=function() return "test" end,iccid=function() return "test" end,imsi=function() return "test" end}
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
assert(#dups>=#frames)

local p=dups[#frames].param
assert(p.pver=="Radar_Top_Track_2.0")
assert(p.target_count==23,"raw parser should accept 23 targets")
assert(p.stable_track_count==23,"all 23 separated moving targets should confirm")
assert(p.raw_target_peak==23 and p.stable_track_peak==23)
assert(p.track_table_count==23 and p.track_table_peak==23)
assert(p.track_table_capacity==32)
assert(p.people_count==23 and p.track_people_count==23)
assert(p.track0.uid==1 and p.track1.uid==2 and p.track2.uid==3)
assert(p.track0.path~="" and p.track0.path_length>=60)
assert(p.track0.direction~=0)
assert(p.track3==nil,"cloud contract must expose only three complete stable tracks")

print("PASS: software parsed and tracked 23 simultaneous targets; cloud exported 3 complete tracks")
