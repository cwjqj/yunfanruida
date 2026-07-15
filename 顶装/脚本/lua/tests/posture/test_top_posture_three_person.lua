-- Posture 3.0 regression: three independent UIDs, filtering, holding and revival.

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
local function tracks(a,b,c,heights)
	return radar_frame(0x82,0x02,{
		record(a,-120, -80,heights[1],8),
		record(b,   0,  20,heights[2],8),
		record(c, 120, 120,heights[3],8)
	})
end

local frames={radar_frame(0x80,0x01,{{1}})}
-- Movement confirms all three software tracks.
for step=0,3 do
	frames[#frames+1]=radar_frame(0x82,0x02,{
		record(1,-180+step*20,-80,170,20),
		record(2, -60+step*20, 20,110,20),
		record(3,  60+step*20,120, 55,20)
	})
end
-- Hold long enough for the cautious initial lying decision.
for _=1,10 do frames[#frames+1]=tracks(1,2,3,{170,110,55}) end
-- One low spike on the standing person must be rejected by the median window.
frames[#frames+1]=tracks(1,2,3,{45,110,55})
for _=1,5 do frames[#frames+1]=tracks(1,2,3,{170,110,55}) end
-- Raw radar IDs jump; software UIDs and postures must remain attached.
for _=1,3 do frames[#frames+1]=tracks(7,8,9,{170,110,55}) end
-- No track frames: all tracks enter holding and posture validity becomes false.
for _=1,3 do frames[#frames+1]=radar_frame(0x01,0x01,{{0x0F}}) end
-- Revival with another set of raw IDs.
for _=1,3 do frames[#frames+1]=tracks(10,11,12,{170,110,55}) end

local source=read_all("versions/posture/radar_iot_task_posture_v3.0.lua")
source=source:gsub("function%s*\n","function __dtu_task()\n",1)
assert(load(source,"@radar_iot_task_posture_v3.0.lua","t",_G))()

local fake_time=3000; local index=1; local waits=0; local dups={}
os.time=function() return fake_time end
mobile={csq=function() return 20 end,imei=function() return "test" end,iccid=function() return "test" end,imsi=function() return "test" end}
log={info=function() end,warn=function() end,error=function() end}
json={encode=function(v) return v end,decode=function() return nil end}
sys={wait=function(ms)
	if ms>=1000 then fake_time=fake_time+math.floor(ms/1000) else fake_time=fake_time+1 end
	waits=waits+1; if waits>#frames+3 then error("__TEST_STOP__") end
end}
function UartStopProRecCh() end
function PronetStopProRecCh() end
function PronetGetRecChAndDel() return nil end
function UartGetRecChAndDel() local v=frames[index]; index=index+1; return v end
function PronetSetSendCh(_,payload) if type(payload)=="table" and payload.cmd=="dup" then dups[#dups+1]=payload end end

local ok,err=pcall(__dtu_task)
assert(not ok and tostring(err):find("__TEST_STOP__",1,true))

local classified=nil
local holding=nil
local revived=nil
local standing_survived_spike=false
for i=1,#dups do
	local p=dups[i].param
	if p and p.pver=="Radar_Top_Posture_3.0" and p.track0 and p.track1 and p.track2 then
		local by_uid={}
		for _,t in ipairs({p.track0,p.track1,p.track2}) do by_uid[t.uid]=t end
		if by_uid[1] and by_uid[2] and by_uid[3]
			and by_uid[1].posture==1 and by_uid[2].posture==2 and by_uid[3].posture==3
			and p.standing_count==1 and p.sitting_count==1 and p.lying_count==1 then
			classified=p
			if p.target0 and p.target0.height==45 and by_uid[1].posture==1 then standing_survived_spike=true end
		end
		if by_uid[1] and by_uid[2] and by_uid[3]
			and by_uid[1].state==2 and by_uid[2].state==2 and by_uid[3].state==2
			and by_uid[1].posture_valid==0 and by_uid[2].posture_valid==0 and by_uid[3].posture_valid==0 then holding=p end
		if by_uid[1] and by_uid[2] and by_uid[3]
			and by_uid[1].radar_id==10 and by_uid[2].radar_id==11 and by_uid[3].radar_id==12
			and by_uid[1].state==1 and by_uid[2].state==1 and by_uid[3].state==1
			and by_uid[1].posture==1 and by_uid[2].posture==2 and by_uid[3].posture==3
			and by_uid[1].posture_valid==1 and by_uid[2].posture_valid==1 and by_uid[3].posture_valid==1 then revived=p end
	end
end

assert(classified,"missing simultaneous standing/sitting/lying report")
assert(standing_survived_spike,"single low height spike changed standing posture")
assert(holding,"holding tracks must retain posture but mark it invalid")
if not revived then
	for i=1,#dups do
		local p=dups[i].param
		if p and p.track0 and (p.track0.radar_id==10 or p.track1.radar_id==10 or p.track2.radar_id==10) then
			for _,t in ipairs({p.track0,p.track1,p.track2}) do
				io.stderr:write(string.format("revive-candidate uid=%s rid=%s state=%s posture=%s valid=%s\n",tostring(t.uid),tostring(t.radar_id),tostring(t.state),tostring(t.posture),tostring(t.posture_valid)))
			end
		end
	end
end
assert(revived,"postures did not survive raw-ID jumps and revival")
assert(revived.target_ids=="1,2,3")
local posture_uids={[revived.posture0.uid]=true,[revived.posture1.uid]=true,[revived.posture2.uid]=true}
assert(posture_uids[1] and posture_uids[2] and posture_uids[3])
assert(revived.primary_target_posture>=1 and revived.primary_target_posture<=3)
assert(revived.posture_event_seq>=3)

print("PASS: three UID-bound postures survived filtering, radar-ID jumps, holding and revival")
