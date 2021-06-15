fid=fopen('voice_pcma_5sec.txt');
[inData,n]=fscanf(fid,'%x');
inData32u=uint32(inData);

rightMask=uint32(sscanf('0000ffff','%x'));
leftMask=uint32(sscanf('ffff0000','%x'));

right16u=uint16(bitand(inData32u,rightMask*ones(n,1,'uint32')));
left16u=uint16(bitshift(bitand(inData32u,leftMask*ones(n,1,'uint32')),-16));

for k=1:1:16,
    rightDelta(k:16:(n-1)*16+k,1) = 1-2*int32(bitget(right16u,17-k));
    leftDelta(k:16:(n-1)*16+k,1) = 1-2*int32(bitget(left16u,17-k));
end

len = 16*n;

deltaMin = int32(10);
deltaMax = int32(1280);
hpara = int32(5);
beta = int32(10);

delta = deltaMin;

%acc = deltaMin*ones(len,1,'int32');
acc = zeros(len,1,'int32');

for i=2:1:len
    if i>=5
        temp = sum(rightDelta(i-4:1:i-1));
        if abs(temp)>=4
            delta = delta+deltaMin;
        else
            delta = bitshift(bitshift(delta,beta)-delta,-beta);
            %delta = delta-delta/2^beta;
        end
    end
    if delta > deltaMax
        delta = deltaMax;
    end
    if delta < deltaMin
        delta = deltaMin;
    end
    accTemp = acc(i-1)+delta*rightDelta(i);
    acc(i) = bitshift(bitshift(accTemp,hpara)-accTemp,-hpara);
    %acc(i) = accTemp-accTemp/2^hpara;
end
rightData = acc;

acc = zeros(len,1,'int32');
for i=2:1:len
    if i>=5
        temp = sum(rightDelta(i-4:1:i-1));
        if abs(temp)>=4
            delta = delta+deltaMin;
        else
            delta = bitshift(bitshift(delta,beta)-delta,-beta);
            %delta = delta-delta/2^beta;
        end
    end
    if delta > deltaMax
        delta = deltaMax;
    end
    if delta < deltaMin
        delta = deltaMin;
    end
    accTemp = acc(i-1)+delta*leftDelta(i);
    acc(i) = bitshift(bitshift(accTemp,hpara)-accTemp,-hpara);
    %acc(i) = accTemp-accTemp/2^hpara;
end
leftData = acc;

stereoData = [leftData,rightData];
soundsc(double(stereoData),2^16)


%rightData=int32(right16u)-int32(int32(right16u)>2^15)*2^16;
%leftData=int32(left16u)-int32(int32(left16u)>2^15)*2^16;
%soundsc(double(rightData))
