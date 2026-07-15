-- Posture 3.0 transition test: hysteresis and sustained stand/sit/lie changes.

local function read_all(path) local f=assert(io.open(path,"rb")); local s=f:read("*a"); f:close(); return s end
local function u16(v) return math.floor(v/256)%256,v%256 end
local function signed16(v) if v<0 then v=0x8000+math.abs(v) end; return u16(v) end
local function record(id,x,h,s)
	local xh,xl=signed16(x); local yh,yl=signed16(0); local hh,hl=u16(h); local sh,sl=signed16(s)
	return {id,1,2,xh,xl,yh,yl,hh,hl,sh,sl}
end
local function frame(ctrl,cmd,records)
	local data={}; for i=1,#records do for j=1,#records[i] do data[#data+1]=records[i][j] end end
	local b={0x53,0x59,ctrl,cmd,math.floor(#data/256),#data%256}; for i=1,#data do b[#b+1]=data[i] end
	local sum=0; for i=1,#b do sum=(sum+b[i])%256 end
	b[#b+1]=sum; b[#b+1]=0x54; b[#b+1]=0x43; return string.char(table.unpack(b))
end
local function track(h) return frame(0x82,0x02,{record(1,0,h,8)}) end

local frames={frame(0x80,0x01,{{1}})}
for step=0,3 do frames[#frames+1]=frame(0x82,0x02,{record(1,-60+step*20,170,20)}) end
for _=1,7 do frames[#frames+1]=track(170) end
-- Boundary noise remains standing because of hysteresis.
for _,h in ipairs({139,141,138,142,140,139}) do frames[#frames+1]=track(h) end
-- Sustained sitting evidence changes posture.
for _=1,12 do frames[#frames+1]=track(105) end
-- 79/81 cm boundary noise remains sitting.
for _,h in ipairs({79,81,78,82,80,79}) do frames[#frames+1]=track(h) end
-- Sustained low height changes to lying.
for _=1,14 do frames[#frames+1]=track(55) end
-- Rise through sitting before standing so Track 2.0 height association stays valid.
for _=1,10 do frames[#frames+1]=track(110) end
for _=1,12 do frames[#frames+1]=track(170) end

local source=read_all("versions/posture/radar_iot_task_posture_v3.0.lua"):gsub("function%s*\n","function __dtu_task()\n",1)
assert(load(source,"@radar_iot_task_posture_v3.0.lua","t",_G))()
local fake_time=5000; local index=1; local waits=0; local dups={}
os.time=function() return fake_time end
mobile={csq=function() return 20 end,imei=function() return "test" end,iccid=function() return "test" end,imsi=function() return "test" end}
log={info=function() end,warn=function() end,error=function() end}; json={encode=function(v) return v end,decode=function() return nil end}
sys={wait=function(ms) fake_time=fake_time+((ms>=1000) and math.floor(ms/1000) or 1); waits=waits+1; if waits>#frames+3 then error("__TEST_STOP__") end end}
function UartStopProRecCh() end; function PronetStopProRecCh() end; function PronetGetRecChAndDel() return nil end
function UartGetRecChAndDel() local v=frames[index]; index=index+1; return v end
function PronetSetSendCh(_,p) if type(p)=="table" and p.cmd=="dup" then dups[#dups+1]=p end end
local ok,err=pcall(__dtu_task); assert(not ok and tostring(err):find("__TEST_STOP__",1,true))

local seen_stand,seen_sit,seen_lie,seen_stand_again=false,false,false,false
local phase=0
for i=1,#dups do
	local p=dups[i].param
	if p and p.track0 and p.track0.uid==1 and p.track0.posture_valid==1 then
		local code=p.track0.posture
		if phase==0 and code==1 then seen_stand=true; phase=1
		elseif phase==1 and code==2 then seen_sit=true; phase=2
		elseif phase==2 and code==3 then seen_lie=true; phase=3
		elseif phase==3 and code==1 then seen_stand_again=true; phase=4 end
	end
end
assert(seen_stand and seen_sit and seen_lie and seen_stand_again,"missing sustained standing/sitting/lying transition sequence")
print("PASS: posture hysteresis ignored boundary noise and confirmed sustained transitions")
