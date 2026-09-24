export const TYPE={RAW:1,XZY:2,ZXZ:3,DIAG:4,CONFIG:5,REST:6,RESULT:7,COMMAND:0x80};
const SOF0=0x43,SOF1=0x44,VERSION=1,HEADER=6,MAX_PAYLOAD=32;

export function crc16(bytes){
  let value=0xffff;
  for(const byte of bytes){
    value^=byte;
    for(let i=0;i<8;i++)value=(value>>>1)^((value&1)?0x8408:0);
  }
  return value&0xffff;
}

export function encodeFrame(type,payload,sequence=0){
  const body=payload instanceof Uint8Array?payload:Uint8Array.from(payload);
  if(body.length>MAX_PAYLOAD)throw new Error("USB 调试载荷过长");
  const frame=new Uint8Array(HEADER+body.length+2);
  frame.set([SOF0,SOF1,VERSION,type,sequence&0xff,body.length]);
  frame.set(body,HEADER);
  const checksum=crc16(frame.subarray(0,frame.length-2));
  frame[frame.length-2]=checksum&0xff;
  frame[frame.length-1]=checksum>>>8;
  return frame;
}

export class StreamDecoder{
  constructor(){this.buffer=[];}
  push(chunk){
    this.buffer.push(...chunk);
    const frames=[];
    while(this.buffer.length>=HEADER+2){
      while(this.buffer.length&&this.buffer[0]!==SOF0)this.buffer.shift();
      if(this.buffer.length<HEADER+2)break;
      if(this.buffer[1]!==SOF1){this.buffer.shift();continue;}
      const length=this.buffer[5];
      if(length>MAX_PAYLOAD){this.buffer.shift();continue;}
      const total=HEADER+length+2;
      if(this.buffer.length<total)break;
      const candidate=Uint8Array.from(this.buffer.slice(0,total));
      const received=candidate[total-2]|candidate[total-1]<<8;
      if(candidate[2]!==VERSION||crc16(candidate.subarray(0,total-2))!==received){this.buffer.shift();continue;}
      this.buffer.splice(0,total);
      frames.push({type:candidate[3],sequence:candidate[4],payload:candidate.slice(HEADER,total-2)});
    }
    return frames;
  }
}
