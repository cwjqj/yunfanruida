-- Posture 3.1: aisle/bed-desk zoning and uncalibrated upper-bed protection.

local function read_all(path) local f=assert(io.open(path,"rb")); local s=f:read("*a"); f:close(); return s end
local function u16(v) return math.floor(v/256)%256,v%256 end
local function signed16(v) if v<0 then v=0x8000+math.abs(v) end; return u16(v) end
local function record(id,x,y,h,s)
	local xh,xl=signed16(x); local yh,yl=signed16(y); local hh,hl=u16(h); local sh,sl=signed16(s)
	return {id,1,2,xh,xl,yh,yl,hh,hl,sh,sl}
end
local function frame(ctrl,cmd,records)
	local data={}; for i=1,#records do for j=1,#records[i] do data[#data+1]=records[i][j] end end
	local b={0x53,0x59,ctrl,cmd,math.floor(#data/256),#data%256}; for i=1,#data do b[#b+1]=data[i] end
	local sum=0; for i=1,#b do sum=(sum+b[i])%256 end
	b[#b+1]=sum; b[#b+1]=0x54; b[#b+1]=0x43; return string.char(table.unpack(b))
end

local frames={frame(0x80,0x01,{{1}})}
for step=0,3 do
	frames[#frames+1]=frame(0x82,0x02,{
		record(1,0,-60+step*20,170,20),       -- aisle standing
		record(2,-120,-60+step*20,170,20),    -- bed zone high/ambiguous
		record(3,120,-60+step*20,110,20)      -- desk sitting
	})
end
for _=1,9 do
	frames[#frames+1]=frame(0x82,0x02,{record(1,0,0,170,8),record(2,-120,0,170,8),record(3,120,0,110,8)})
end
-- Exact X boundary diagnostics: 74 is aisle; 75 and 165 are bed/desk.
frames[#frames+1]=frame(0x82,0x02,{record(11,74,0,170,8),record(12,75,0,110,8),record(13,165,0,110,8)})

local source=read_all("radar_iot_task_posture.lua"):gsub("function%s*\n","function __dtu_task()\n",1)
assert(load(source,"@radar_iot_task_posture.lua","t",_G))()
local fake_time=7000; local index=1; local waits=0; local dups={}
os.time=function() return fake_time end
mobile={csq=function() return 20 end,imei=function() return "test" end,iccid=function() return "test" end,imsi=function() return "test" end}
log={info=function() end,warn=function() end,error=function() end}; json={encode=function(v) return v end,decode=function() return nil end}
sys={wait=function(ms) fake_time=fake_time+((ms>=1000) and math.floor(ms/1000) or 1); waits=waits+1; if waits>#frames+3 then error("__TEST_STOP__") end end}
function UartStopProRecCh() end; function PronetStopProRecCh() end; function PronetGetRecChAndDel() return nil end
function UartGetRecChAndDel() local v=frames[index]; index=index+1; return v end
function PronetSetSendCh(_,p) if type(p)=="table" and p.cmd=="dup" then dups[#dups+1]=p end end
local ok,err=pcall(__dtu_task); assert(not ok and tostring(err):find("__TEST_STOP__",1,true))

local zoned=nil; local boundary=false
for i=1,#dups do
	local p=dups[i].param
	if p and p.pver=="Radar_Top_Posture_3.1" and p.track0 and p.track1 and p.track2 then
		local by_uid={}; for _,t in ipairs({p.track0,p.track1,p.track2}) do by_uid[t.uid]=t end
		if by_uid[1] and by_uid[2] and by_uid[3]
			and by_uid[1].posture_zone==1 and by_uid[1].posture==1 and by_uid[1].posture_valid==1
			and by_uid[2].posture_zone==2 and by_uid[2].in_bed_zone==1 and by_uid[2].posture_valid==0 and by_uid[2].posture_reason=="bed_profile_unconfigured"
			and by_uid[3].posture_zone==2 and by_uid[3].posture==2 and by_uid[3].posture_valid==1
			and p.aisle_track_count==1 and p.bed_desk_track_count==2 and p.unknown_posture_count==1
			and p.bed_profile_ready==0 and p.bed_surface_height==0 then zoned=p end
		if p.target0 and p.target1 and p.target2 and p.target0.x==74 and p.target0.zone==1
			and p.target1.x==75 and p.target1.zone==2 and p.target1.in_bed_zone==1
			and p.target2.x==165 and p.target2.zone==2 and p.target2.in_bed_zone==1 then boundary=true end
	end
end
assert(zoned,"missing conservative aisle/bed-desk posture report")
assert(boundary,"X=75/165 bed-desk boundaries were not applied exactly")
print("PASS: Posture 3.1 applied aisle/bed-desk zones and protected uncalibrated upper-bed heights")
