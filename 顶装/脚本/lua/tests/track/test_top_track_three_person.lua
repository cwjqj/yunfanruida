-- Three-person continuity test: confirmation, radar-ID jump, holding and revival.

local function read_all(path)
	local f=assert(io.open(path,"rb")); local s=f:read("*a"); f:close(); return s
end
local function u16(v) return math.floor(v/256)%256,v%256 end
local function signed16(v) if v<0 then v=0x8000+math.abs(v) end; return u16(v) end
local function record(id,x,y,h,s)
	local xh,xl=signed16(x); local yh,yl=signed16(y); local hh,hl=u16(h); local sh,sl=signed16(s)
	return {id,1,2,xh,xl,yh,yl,hh,hl,sh,sl}
end
local function radar_frame(ctrl,cmd,records)
	local data={}
	for i=1,#records do for j=1,#records[i] do data[#data+1]=records[i][j] end end
	local bytes={0x53,0x59,ctrl,cmd,math.floor(#data/256),#data%256}
	for i=1,#data do bytes[#bytes+1]=data[i] end
	local sum=0; for i=1,#bytes do sum=(sum+bytes[i])%256 end
	bytes[#bytes+1]=sum; bytes[#bytes+1]=0x54; bytes[#bytes+1]=0x43
	return string.char(table.unpack(bytes))
end

local frames={}
for step=0,3 do
	frames[#frames+1]=radar_frame(0x82,0x02,{
		record(1,-140+step*30,-60,170,20),
		record(2,-45+step*30,20,165,20),
		record(3,50+step*30,100,175,20)
	})
end
frames[#frames+1]=radar_frame(0x82,0x02,{
	record(7,-40,-60,170,10),record(8,55,20,165,10),record(9,150,100,175,10)
})
for _=1,3 do frames[#frames+1]=radar_frame(0x01,0x01,{{0x0F}}) end
frames[#frames+1]=radar_frame(0x82,0x02,{
	record(10,-35,-60,170,8),record(11,60,20,165,8),record(12,145,100,175,8)
})

local source=read_all("radar_iot_task_track.lua")
source=source:gsub("function%s*\n","function __dtu_task()\n",1)
assert(load(source,"@radar_iot_task_track.lua","t",_G))()

local fake_time=2000; local index=1; local waits=0; local dups={}
os.time=function() return fake_time end
mobile={csq=function() return 20 end,imei=function() return "test" end,iccid=function() return "test" end,imsi=function() return "test" end}
log={info=function() end,warn=function() end,error=function() end}
json={encode=function(v) return v end,decode=function() return nil end}
sys={wait=function(ms)
	if ms>=1000 then fake_time=fake_time+math.floor(ms/1000) else fake_time=fake_time+1 end
	waits=waits+1; if waits>#frames+2 then error("__TEST_STOP__") end
end}
function UartStopProRecCh() end
function PronetStopProRecCh() end
function PronetGetRecChAndDel() return nil end
function UartGetRecChAndDel() local v=frames[index]; index=index+1; return v end
function PronetSetSendCh(_,payload) if type(payload)=="table" and payload.cmd=="dup" then dups[#dups+1]=payload end end

local ok,err=pcall(__dtu_task)
assert(not ok and tostring(err):find("__TEST_STOP__",1,true))
local p=nil
for i=1,#dups do
	local candidate=dups[i].param
	if candidate.track0 and candidate.track1 and candidate.track2
		and candidate.track0.radar_id==10
		and candidate.track1.radar_id==11
		and candidate.track2.radar_id==12
		and candidate.track0.state==1
		and candidate.track1.state==1
		and candidate.track2.state==1 then
		p=candidate
	end
end
assert(p,"missing active revival report for radar IDs 10/11/12")
assert(p.people_count==3 and p.stable_track_count==3)
assert(p.target_ids=="1,2,3","internal UIDs must survive raw radar-ID changes")
assert(p.track0.uid==1 and p.track1.uid==2 and p.track2.uid==3)
assert(p.track0.radar_id==10 and p.track1.radar_id==11 and p.track2.radar_id==12)
assert(p.track0.state==1 and p.track1.state==1 and p.track2.state==1)
assert(p.track0.path_length>=60 and p.track1.path_length>=60 and p.track2.path_length>=60)
assert(p.track0.path~="" and p.track1.path~="" and p.track2.path~="")
assert(p.track_event:find("revived",1,true),"returning tracks should emit a revived event")

print("PASS: three stable UIDs survived radar-ID jumps, holding loss and revival with complete paths")
