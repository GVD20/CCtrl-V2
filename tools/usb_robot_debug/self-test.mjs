import { compose, maxMatrixError } from "./rotation-math.mjs";
import { encodeFrame, StreamDecoder, TYPE } from "./protocol.mjs";

const payload=Uint8Array.from([0x10,1,0,...Array(12).fill(0),0x15]);
const frame=encodeFrame(TYPE.COMMAND,payload,7);
const decoder=new StreamDecoder();
const decoded=[...decoder.push(frame.slice(0,5)),...decoder.push(frame.slice(5))];
if(decoded.length!==1||decoded[0].type!==TYPE.COMMAND||decoded[0].sequence!==7||decoded[0].payload.length!==16)throw new Error("USB frame round trip");
const xzy=compose("XZY",[.2,-.4,.7]),zxz=compose("ZXZ",[.1,.5,-.3]);
if(!Number.isFinite(maxMatrixError(xzy,zxz)))throw new Error("render matrices");
console.log("USB serial frame round trip: OK");
console.log("XZY/ZXZ device-result rendering inputs: OK");
