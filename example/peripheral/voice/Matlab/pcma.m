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

right16u(1:2:2*n-1,1)=uint16(bin2dec(fliplr(dec2bin(rightEarly16u,8))));
right16u(2:2:2*n,1)=uint16(bin2dec(fliplr(dec2bin(rightLate16u,8))));
left16u(1:2:2*n-1,1)=uint16(bin2dec(fliplr(dec2bin(leftEarly16u,8))));
left16u(2:2:2*n,1)=uint16(bin2dec(fliplr(dec2bin(leftLate16u,8))));

right16u(1:2:2*n-1,1)=uint16(bin2dec(fliplr(dec2bin(rightLate16u,8))));
right16u(2:2:2*n,1)=uint16(bin2dec(fliplr(dec2bin(rightEarly16u,8))));
left16u(1:2:2*n-1,1)=uint16(bin2dec(fliplr(dec2bin(leftLate16u,8))));
left16u(2:2:2*n,1)=uint16(bin2dec(fliplr(dec2bin(leftEarly16u,8))));

quantMask=uint16(sscanf('000f','%x'));
segMask=uint16(sscanf('0070','%x'));
signMask=uint16(sscanf('0080','%x'));
xorMask=uint16(sscanf('0055','%x'));
bias1=uint16(sscanf('0008','%x'));
bias2=uint16(sscanf('0100','%x'));

rightBase=bitshift(bitand(bitxor(right16u,xorMask*ones(2*n,1,'uint16')),quantMask*ones(2*n,1,'uint16')),4);
leftBase=bitshift(bitand(bitxor(left16u,xorMask*ones(2*n,1,'uint16')),quantMask*ones(2*n,1,'uint16')),4);
rightShift=bitshift(bitand(bitxor(right16u,xorMask*ones(2*n,1,'uint16')),segMask*ones(2*n,1,'uint16')),-4);
leftShift=bitshift(bitand(bitxor(left16u,xorMask*ones(2*n,1,'uint16')),segMask*ones(2*n,1,'uint16')),-4);
rightSign=bitshift(bitand(bitxor(right16u,xorMask*ones(2*n,1,'uint16')),signMask*ones(2*n,1,'uint16')),-7);
leftSign=bitshift(bitand(bitxor(left16u,xorMask*ones(2*n,1,'uint16')),signMask*ones(2*n,1,'uint16')),-7);
%rightBase=bitshift(bitand((right16u),quantMask*ones(2*n,1,'uint16')),3)+bias;
%leftBase=bitshift(bitand((left16u),quantMask*ones(2*n,1,'uint16')),3)+bias;
%rightShift=bitshift(bitand((right16u),segMask*ones(2*n,1,'uint16')),-4);
%leftShift=bitshift(bitand((left16u),segMask*ones(2*n,1,'uint16')),-4);
%rightSign=bitshift(bitand((right16u),signMask*ones(2*n,1,'uint16')),-7);
%leftSign=bitshift(bitand((left16u),signMask*ones(2*n,1,'uint16')),-7);


rightData=int32(bitshift(rightBase+bias1+uint16(rightShift>0)*bias2,rightShift-uint16(rightShift>0))).*(1-2*int32(rightSign));
leftData=int32(bitshift(leftBase+bias1+uint16(leftShift>0)*bias2,leftShift-uint16(leftShift>0))).*(1-2*int32(leftSign));

%rightData=int32(right16u)-int32(int32(right16u)>2^15)*2^16;
%leftData=int32(left16u)-int32(int32(left16u)>2^15)*2^16;
soundsc(double(rightData))
