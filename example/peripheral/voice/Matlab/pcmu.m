fid=fopen('voice_cvsd_5sec.txt');
[inData,n]=fscanf(fid,'%x');
inData32u=uint32(inData);

rightLateMask=uint32(sscanf('000000ff','%x'));
rightEarlyMask=uint32(sscanf('0000ff00','%x'));
leftLateMask=uint32(sscanf('00ff0000','%x'));
leftEarlyMask=uint32(sscanf('ff000000','%x'));
rightLate16u=uint16(bitand(inData32u,rightLateMask*ones(n,1,'uint32')));
rightEarly16u=uint16(bitshift(bitand(inData32u,rightEarlyMask*ones(n,1,'uint32')),-8));
leftLate16u=uint16(bitshift(bitand(inData32u,leftLateMask*ones(n,1,'uint32')),-16));
leftEarly16u=uint16(bitshift(bitand(inData32u,leftEarlyMask*ones(n,1,'uint32')),-24));

right16u(1:2:2*n-1,1)=rightEarly16u;
right16u(2:2:2*n,1)=rightLate16u;
left16u(1:2:2*n-1,1)=leftEarly16u;
left16u(2:2:2*n,1)=leftLate16u;

quantMask=uint16(sscanf('000f','%x'));
segMask=uint16(sscanf('0070','%x'));
signMask=uint16(sscanf('0080','%x'));
bias=uint16(sscanf('0084','%x'));

rightBase=bitshift(bitand(bitcmp(right16u),quantMask*ones(2*n,1,'uint16')),3)+bias;
leftBase=bitshift(bitand(bitcmp(left16u),quantMask*ones(2*n,1,'uint16')),3)+bias;
rightShift=bitshift(bitand(bitcmp(right16u),segMask*ones(2*n,1,'uint16')),-4);
leftShift=bitshift(bitand(bitcmp(left16u),segMask*ones(2*n,1,'uint16')),-4);
rightSign=bitshift(bitand(bitcmp(right16u),signMask*ones(2*n,1,'uint16')),-7);
leftSign=bitshift(bitand(bitcmp(left16u),signMask*ones(2*n,1,'uint16')),-7);

rightData=int32(bitshift(rightBase,rightShift)-bias).*(1-2*int32(rightSign));
leftData=int32(bitshift(leftBase,leftShift)-bias).*(1-2*int32(leftSign));

stereoData=[leftData,rightData];

soundsc(double(rightData))
