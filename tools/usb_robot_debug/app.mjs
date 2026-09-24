import { compose, degToRad, multiply, rotation } from "./rotation-math.mjs";
import { encodeFrame, StreamDecoder, TYPE } from "./protocol.mjs";

const $=selector=>document.querySelector(selector);
const colors={X:"#e44d4d",Y:"#27a85f",Z:"#3978dd"};
const state={port:null,reader:null,connected:false,closing:false,decoder:new StreamDecoder(),txSequence:0,raw:[0,0,0,0,0,0],comp:[0,228,683,0,0,0],xzy:[0,228,683,0,0,0],zxz:[0,228,683,0,0,0],offset:[0,0,0,0,0,0],dir:[1,1,1,1,1,1],revision:0,valid:0,sequence:0,diag:{status:0,errors:0,warnings:0,main:0,handle:0,flags:0,joyX:0,joyY:0,trigger:0,latency:0,lost:0},rest:{revision:0,raw:[0,0,0,0,0,0],flags:2},frameTimes:[],viewYaw:-.72,viewPitch:.34,zoom:4.5,demoTimer:null,demoIndex:0};
const demoFrames=[
  {raw:[0,228,683,0,0,0],comp:[0,228,683,0,0,0],xzy:[0,228,683,0,0,0],zxz:[0,228,683,0,0,0]},
  {raw:[205,540,23,284,3890,546],comp:[205,540,23,284,-206,546],xzy:[205,540,23,-284,-206,-546],zxz:[205,540,23,-1473,845,1396]},
  {raw:[3698,774,3755,3470,432,1195],comp:[-398,774,-341,-626,432,1195],xzy:[-398,774,-341,626,432,-1195],zxz:[-398,774,-341,1042,801,-971]},
];

function countsToDegree(value){return value*360/4096;}
function signedCount(value){return value>=2048?value-4096:value;}
function fmtDegree(value){let clean=Math.abs(value)<.005?0:value;if(Math.abs(Math.abs(clean)-180)<.005)clean=180;return `${clean>=0?"+":""}${clean.toFixed(2)}°`;}
function hex(value){return (value&0xff).toString(16).padStart(2,"0").toUpperCase();}
function setStatus(text,kind=""){$("#statusText").textContent=text;$("#statusDot").className=kind;}
async function sendCommand(payload){if(!state.connected||!state.port?.writable)throw new Error("请先连接设备");const writer=state.port.writable.getWriter();try{await writer.write(encodeFrame(TYPE.COMMAND,payload,state.txSequence++));}finally{writer.releaseLock();}}

const rows=[];
for(let i=0;i<6;i++){
  const tr=document.createElement("tr");
  tr.innerHTML=`<td class="joint-name">A${i+1}</td><td><span class="sensor-dot"></span><em>无数据</em></td><td><strong class="raw-count">0</strong><br><small class="raw-degree">0.00°</small></td><td><input class="offset" type="number" min="0" max="4095" step="1" value="0"></td><td><select class="direction"><option value="1">正向</option><option value="-1">反向</option></select></td><td class="angle-value">+0.00°</td><td><div class="preset-row"><button data-target="0">0°</button><button data-target="-90">−90°</button><button data-target="90">90°</button><button data-target="180">180°</button><button data-custom>自定</button></div></td>`;
  $("#jointRows").appendChild(tr);
  const row={tr,dot:tr.querySelector(".sensor-dot"),status:tr.querySelector("em"),rawCount:tr.querySelector(".raw-count"),rawDegree:tr.querySelector(".raw-degree"),offset:tr.querySelector(".offset"),direction:tr.querySelector(".direction"),angle:tr.querySelector(".angle-value")};
  row.offset.addEventListener("input",()=>{state.offset[i]=Math.max(0,Math.min(4095,Number(row.offset.value)||0));updateCalibrationPreview();});
  row.direction.addEventListener("change",()=>{state.dir[i]=Number(row.direction.value);updateCalibrationPreview();});
  tr.querySelectorAll("[data-target]").forEach(button=>button.addEventListener("click",()=>calibrateAxis(i,Number(button.dataset.target))));
  tr.querySelector("[data-custom]").addEventListener("click",()=>{const value=prompt(`将 A${i+1} 当前位姿校准到多少度？`,`0`);if(value!==null&&Number.isFinite(Number(value)))calibrateAxis(i,Number(value));});
  rows.push(row);
}

function calibrateAxis(axis,degree){let normalized=((degree+180)%360+360)%360-180;if(Math.abs(normalized+180)<1e-9)normalized=180;const target=Math.round(normalized*4096/360);state.offset[axis]=state.dir[axis]>0?(state.raw[axis]-target)&0xfff:(state.raw[axis]+target)&0xfff;rows[axis].offset.value=state.offset[axis];updateCalibrationPreview();}
function previewCount(axis){let delta=(state.raw[axis]-state.offset[axis])&0xfff;if(state.dir[axis]<0)delta=(4096-delta)&0xfff;return signedCount(delta);}
function updateCalibrationPreview(){state.comp=state.raw.map((_,i)=>previewCount(i));rows.forEach((row,i)=>{const valid=Boolean(state.valid&(1<<i));row.dot.classList.toggle("valid",valid);row.status.textContent=valid?"有效":"无效";row.rawCount.textContent=state.raw[i];row.rawDegree.textContent=`${(state.raw[i]*360/4096).toFixed(2)}°`;row.angle.textContent=fmtDegree(countsToDegree(state.comp[i]));});}
function syncConfigControls(){rows.forEach((row,i)=>{row.offset.value=state.offset[i];row.direction.value=String(state.dir[i]);});updateCalibrationPreview();}

function dataView(payload){return new DataView(payload.buffer,payload.byteOffset,payload.byteLength);}
function parsePose(frame,signed=false){if(frame.payload.length!==13)throw new Error("姿态包格式错误");const view=dataView(frame.payload);return{sequence:frame.sequence,valid:view.getUint8(0),values:Array.from({length:6},(_,i)=>signed?view.getInt16(1+i*2,true):view.getUint16(1+i*2,true))};}
function parseConfig(payload){if(payload.length!==15)throw new Error("校准配置格式错误");const view=dataView(payload);state.revision=view.getUint16(0,true);state.offset=Array.from({length:6},(_,i)=>view.getUint16(2+i*2,true));const mask=view.getUint8(14);state.dir=Array.from({length:6},(_,i)=>(mask&(1<<i))?-1:1);syncConfigControls();}
function parseRest(payload){if(payload.length!==15)throw new Error("休息位姿格式错误");const view=dataView(payload);state.rest.revision=view.getUint16(0,true);state.rest.raw=Array.from({length:6},(_,i)=>view.getUint16(2+i*2,true));state.rest.flags=view.getUint8(14);updateDiagnostics();}
function parseDiag(payload){if(payload.length!==17)throw new Error("诊断包格式错误");const view=dataView(payload);state.diag={status:view.getUint8(0),errors:view.getUint8(1),warnings:view.getUint8(2),main:view.getUint8(4),handle:view.getUint8(5),flags:view.getUint8(6),joyX:view.getUint16(7,true),joyY:view.getUint16(9,true),trigger:view.getUint16(11,true),latency:view.getUint16(13,true),lost:view.getUint16(15,true)};updateDiagnostics();}

function onRaw(frame){const packet=parsePose(frame);state.raw=packet.values;state.valid=packet.valid;state.sequence=packet.sequence;const now=performance.now();state.frameTimes.push(now);while(state.frameTimes.length&&now-state.frameTimes[0]>2000)state.frameTimes.shift();$("#rateValue").textContent=(state.frameTimes.length>1?(state.frameTimes.length-1)*1000/(state.frameTimes.at(-1)-state.frameTimes[0]):0).toFixed(1);updateCalibrationPreview();updateHeader();}
function onResult(payload){if(payload.length!==7)return;const operation=payload[0],status=payload[1];setStatus(status===0?`命令 0x${operation.toString(16)} 已完成`:`命令 0x${operation.toString(16)} 被拒绝`,status===0?"ok":"error");}
function dispatchFrame(frame){switch(frame.type){case TYPE.RAW:onRaw(frame);break;case TYPE.XZY:state.xzy=parsePose(frame,true).values;break;case TYPE.ZXZ:state.zxz=parsePose(frame,true).values;break;case TYPE.DIAG:parseDiag(frame.payload);break;case TYPE.CONFIG:parseConfig(frame.payload);break;case TYPE.REST:parseRest(frame.payload);break;case TYPE.RESULT:onResult(frame.payload);break;}}

function updateHeader(){$("#seqValue").textContent=String(state.sequence).padStart(3,"0");$("#validValue").textContent=hex(state.valid);}
function updateDiagnostics(){const d=state.diag,statusNames=["INIT","ACTIVE","DEGRADED","DISCONNECTED"];$("#controllerState").textContent=statusNames[d.status]??String(d.status);$("#warningValue").textContent=hex(d.warnings);$("#mainButtons").textContent=(d.main&255).toString(2).padStart(8,"0");$("#handleButtons").textContent=(d.handle&15).toString(2).padStart(4,"0");$("#joystickValue").textContent=`${d.joyX} / ${d.joyY}`;$("#triggerValue").textContent=d.trigger;$("#latencyValue").textContent=d.latency;$("#lostValue").textContent=d.lost;const inRest=Boolean(d.flags&1),guardEnabled=Boolean(d.flags&2),guardActive=Boolean(d.flags&4);$("#errorPill").textContent=`ERROR ${hex(d.errors)}`;$("#errorPill").classList.toggle("error",d.errors!==0);$("#restPill").textContent=`休息区域：${inRest?"是":"否"}`;$("#guardPill").textContent=`上电保护：${guardActive?"激活":guardEnabled?"待下次上电":"关闭"}`;$("#guardToggle").checked=guardEnabled;}

async function readDeviceState(){if(state.connected)await sendCommand([0x01]);}
async function readLoop(){
  while(state.connected&&state.port?.readable){
    state.reader=state.port.readable.getReader();
    try{while(state.connected){const {value,done}=await state.reader.read();if(done)break;if(value)for(const frame of state.decoder.push(value))dispatchFrame(frame);}}
    catch(error){if(!state.closing)setStatus(`串口读取失败：${error.message}`,"error");}
    finally{state.reader.releaseLock();state.reader=null;}
  }
  if(!state.closing&&state.connected)await disconnect();
}
async function connect(){try{if(state.demoTimer){clearInterval(state.demoTimer);state.demoTimer=null;}setStatus("选择 CCtrl USB 串口…");const port=await navigator.serial.requestPort();await port.open({baudRate:2000000,bufferSize:4096});state.port=port;state.decoder=new StreamDecoder();state.connected=true;state.closing=false;$("#connectBtn").textContent="断开";$("#usbValue").textContent="Web Serial";setStatus("已连接","ok");readLoop();await readDeviceState();}catch(error){if(error.name!=="NotFoundError")setStatus(`连接失败：${error.message}`,"error");}}
async function disconnect(){state.closing=true;state.connected=false;try{await state.reader?.cancel();}catch{}try{await state.port?.close();}catch{}state.reader=null;state.port=null;state.closing=false;$("#connectBtn").textContent="连接设备";$("#usbValue").textContent="未连接";setStatus("未连接");}

async function saveCalibration(){if(!state.connected)return setStatus("请先连接设备","error");const payload=new Uint8Array(16),view=dataView(payload);payload[0]=0x10;view.setUint16(1,state.revision,true);state.offset.forEach((value,i)=>view.setUint16(3+i*2,value,true));let mask=0;state.dir.forEach((value,i)=>{if(value<0)mask|=1<<i;});payload[15]=mask;await sendCommand(payload);setStatus("校准配置已上传，等待设备确认…");}
async function sendRestCommand(bytes){if(!state.connected)return setStatus("请先连接设备","error");await sendCommand(bytes);}

$("#connectBtn").addEventListener("click",()=>state.connected?disconnect():connect());
$("#saveBtn").addEventListener("click",()=>saveCalibration().catch(error=>setStatus(`保存失败：${error.message}`,"error")));
$("#reloadBtn").addEventListener("click",()=>readDeviceState().catch(error=>setStatus(`读取失败：${error.message}`,"error")));
$("#setRestBtn").addEventListener("click",()=>{if(confirm("将当前六路编码器原始值保存为新的休息位姿？"))sendRestCommand([0x20]).catch(error=>setStatus(`保存失败：${error.message}`,"error"));});
$("#guardToggle").addEventListener("change",event=>sendRestCommand([0x21,event.target.checked?1:0]).catch(error=>setStatus(`设置失败：${error.message}`,"error")));
$("#demoBtn").addEventListener("click",async()=>{await disconnect();const tick=()=>{const frame=demoFrames[state.demoIndex++%demoFrames.length];Object.assign(state,{raw:[...frame.raw],comp:[...frame.comp],xzy:[...frame.xzy],zxz:[...frame.zxz],valid:0x3f,sequence:(state.sequence+1)&255});state.diag={status:1,errors:0,warnings:0,main:state.demoIndex&3,handle:(state.demoIndex<<1)&15,flags:3,joyX:2048,joyY:2048,trigger:1024,latency:2630,lost:0};updateCalibrationPreview();updateHeader();updateDiagnostics();};tick();state.demoTimer=setInterval(tick,900);setStatus("演示数据","ok");$("#usbValue").textContent="DEMO";});
if(!("serial" in navigator)){$("#connectBtn").disabled=true;setStatus("当前浏览器不支持 Web Serial","error");}

const add=(a,b)=>[a[0]+b[0],a[1]+b[1],a[2]+b[2]],scale=(v,s)=>v.map(x=>x*s),transform=(m,v)=>m.map(r=>r[0]*v[0]+r[1]*v[1]+r[2]*v[2]);
function cameraMatrix(){return multiply(rotation("X",state.viewPitch),rotation("Y",state.viewYaw));}
function project(point,w,h){const [x,y,z]=transform(cameraMatrix(),point),focal=Math.min(w,h)*1.08,depth=state.zoom-z;return[w/2+x*focal/depth,h*.55-y*focal/depth];}
function line(ctx,a,b,w,h,color,width=1,alpha=1,dash=[]){const p=project(a,w,h),q=project(b,w,h);ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...q);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.globalAlpha=alpha;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);ctx.globalAlpha=1;}
function polyline(ctx,points,w,h,color,width=1,alpha=1){ctx.beginPath();points.forEach((point,i)=>{const p=project(point,w,h);i?ctx.lineTo(...p):ctx.moveTo(...p);});ctx.strokeStyle=color;ctx.lineWidth=width;ctx.globalAlpha=alpha;ctx.stroke();ctx.globalAlpha=1;}
function circle(axis,frame,radius,center){const points=[];for(let i=0;i<=64;i++){const t=i/64*Math.PI*2,p=axis==="X"?[0,Math.cos(t)*radius,Math.sin(t)*radius]:axis==="Y"?[Math.cos(t)*radius,0,Math.sin(t)*radius]:[Math.cos(t)*radius,Math.sin(t)*radius,0];points.push(add(center,transform(frame,p)));}return points;}
function arrow(ctx,origin,vector,w,h,color,label,width=2){const end=add(origin,vector),p=project(origin,w,h),q=project(end,w,h);line(ctx,origin,end,w,h,color,width);const angle=Math.atan2(q[1]-p[1],q[0]-p[0]);ctx.beginPath();ctx.moveTo(...q);ctx.lineTo(q[0]-8*Math.cos(angle-.45),q[1]-8*Math.sin(angle-.45));ctx.lineTo(q[0]-8*Math.cos(angle+.45),q[1]-8*Math.sin(angle+.45));ctx.closePath();ctx.fillStyle=color;ctx.fill();ctx.font="700 10px Cascadia Mono,monospace";ctx.fillText(label,q[0]+4,q[1]-4);}
function joint(ctx,point,w,h,label){const p=project(point,w,h);ctx.beginPath();ctx.arc(p[0],p[1],5,0,Math.PI*2);ctx.fillStyle="#fff";ctx.fill();ctx.strokeStyle="#52606d";ctx.lineWidth=2;ctx.stroke();ctx.fillStyle="#52606d";ctx.font="700 8px system-ui";ctx.fillText(label,p[0]+8,p[1]-6);}
function grid(ctx,w,h){for(let i=-4;i<=4;i++){line(ctx,[-2,-1,i*.5],[2,-1,i*.5],w,h,"#dfe4e8",i?1:1.4,i?.7:1);line(ctx,[i*.5,-1,-2],[i*.5,-1,2],w,h,"#dfe4e8",i?1:1.4,i?.7:1);}}
function armKinematics(){const raw=state.xzy.slice(0,3).map(v=>degToRad(countsToDegree(v))),a1=-raw[0],a2=raw[1]-Math.PI,a3=-(raw[2]-Math.PI),shoulder=[0,-1,0],base=rotation("Y",a1),upper=multiply(base,rotation("X",a2)),elbow=add(shoulder,transform(upper,[0,0,1])),forearm=multiply(upper,rotation("X",a3)),wrist=add(elbow,transform(forearm,[0,0,1]));return{shoulder,elbow,wrist,base,upper,forearm};}
function render(canvas,order,counts,arm){const wristAngles=counts.slice(3).map(v=>degToRad(countsToDegree(v))),target=compose(order,wristAngles),rect=canvas.getBoundingClientRect(),dpr=Math.min(devicePixelRatio||1,2);if(canvas.width!==Math.round(rect.width*dpr)||canvas.height!==Math.round(rect.height*dpr)){canvas.width=Math.round(rect.width*dpr);canvas.height=Math.round(rect.height*dpr);}const ctx=canvas.getContext("2d");ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,rect.width,rect.height);ctx.lineCap="round";ctx.lineJoin="round";grid(ctx,rect.width,rect.height);polyline(ctx,circle("Y",compose("XYZ",[0,0,0]),.38,arm.shoulder),rect.width,rect.height,colors.Y,2,.7);polyline(ctx,circle("X",arm.base,.33,arm.shoulder),rect.width,rect.height,colors.X,2,.7);line(ctx,arm.shoulder,arm.elbow,rect.width,rect.height,"#24313d",9);line(ctx,arm.shoulder,arm.elbow,rect.width,rect.height,"#7fa8ff",4);polyline(ctx,circle("X",arm.upper,.3,arm.elbow),rect.width,rect.height,colors.X,2,.7);line(ctx,arm.elbow,arm.wrist,rect.width,rect.height,"#24313d",9);line(ctx,arm.elbow,arm.wrist,rect.width,rect.height,"#8bd5bd",4);joint(ctx,arm.shoulder,rect.width,rect.height,"A1/A2");joint(ctx,arm.elbow,rect.width,rect.height,"A3");joint(ctx,arm.wrist,rect.width,rect.height,"WRIST");let prefix=arm.forearm;order.split("").forEach((axis,i)=>{polyline(ctx,circle(axis,prefix,.5-i*.075,arm.wrist),rect.width,rect.height,colors[axis],2,.62+i*.12);prefix=multiply(prefix,rotation(axis,wristAngles[i]));});const final=multiply(arm.forearm,target);[["X",[1,0,0]],["Y",[0,1,0]],["Z",[0,0,1]]].forEach(([axis,unit])=>arrow(ctx,arm.wrist,scale(transform(final,unit),.58),rect.width,rect.height,colors[axis],`${axis}′`,2));}
function valueRows(host,labels,values){host.innerHTML=labels.map((label,i)=>`<div>${label}<strong>${fmtDegree(countsToDegree(values[i+3]))}</strong></div>`).join("");}
function draw(){const arm=armKinematics();render($("#xzyCanvas"),"XZY",state.xzy,arm);render($("#zxzCanvas"),"ZXZ",state.zxz,arm);valueRows($("#xzyValues"),["A4 / X","A5 / Z","A6 / Y"],state.xzy);valueRows($("#zxzValues"),["A4 / α·Z","A5 / β·X","A6 / γ·Z"],state.zxz);requestAnimationFrame(draw);}draw();
