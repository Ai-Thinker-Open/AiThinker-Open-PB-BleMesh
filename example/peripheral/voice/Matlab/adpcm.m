indexTable = [
-1, -1, -1, -1, 2, 4, 6, 8, ...
-1, -1, -1, -1, 2, 4, 6, 8
];

stepsizeTable = [
7, 8, 9, 10, 11, 12, 13, 14, 16, 17, ...
19, 21, 23, 25, 28, 31, 34, 37, 41, 45, ...
50, 55, 60, 66, 73, 80, 88, 97, 107, 118, ...
130, 143, 157, 173, 190, 209, 230, 253, 279, 307, ...
337, 371, 408, 449, 494, 544, 598, 658, 724, 796, ...
876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, ...
2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, ...
5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, ...
15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
];

fid=fopen('voice_adpcm_10sec.txt');
[inData,n]=fscanf(fid,'%x');
inData32u=uint32(inData);

right4Mask = uint32(sscanf('0000000f','%x'));
right3Mask = uint32(sscanf('000000f0','%x'));
right2Mask = uint32(sscanf('00000f00','%x'));
right1Mask = uint32(sscanf('0000f000','%x'));
left4Mask = uint32(sscanf('000f0000','%x'));
left3Mask = uint32(sscanf('00f00000','%x'));
left2Mask = uint32(sscanf('0f000000','%x'));
left1Mask = uint32(sscanf('f0000000','%x'));

right4Delta = uint8(bitand(inData32u,right4Mask*ones(n,1,'uint32')));
right3Delta = uint8(bitshift(bitand(inData32u,right3Mask*ones(n,1,'uint32')),-4));
right2Delta = uint8(bitshift(bitand(inData32u,right2Mask*ones(n,1,'uint32')),-8));
right1Delta = uint8(bitshift(bitand(inData32u,right1Mask*ones(n,1,'uint32')),-12));
left4Delta = uint8(bitshift(bitand(inData32u,left4Mask*ones(n,1,'uint32')),-16));
left3Delta = uint8(bitshift(bitand(inData32u,left3Mask*ones(n,1,'uint32')),-20));
left2Delta = uint8(bitshift(bitand(inData32u,left2Mask*ones(n,1,'uint32')),-24));
left1Delta = uint8(bitshift(bitand(inData32u,left1Mask*ones(n,1,'uint32')),-28));

rightDelta(1:4:4*n-3,1)=right1Delta;
rightDelta(2:4:4*n-2,1)=right2Delta;
rightDelta(3:4:4*n-1,1)=right3Delta;
rightDelta(4:4:4*n,1)=right4Delta;
leftDelta(1:4:4*n-3,1)=left1Delta;
leftDelta(2:4:4*n-2,1)=left2Delta;
leftDelta(3:4:4*n-1,1)=left3Delta;
leftDelta(4:4:4*n,1)=left4Delta;

m = 4*n;

delta = rightDelta;
data = zeros(m,1);
index = 0;
predictedSample = 0;
for i = 1:1:m,
    step = stepsizeTable(index+1);
    diffPredict = bitshift(step,-3);
    if bitget(delta(i),3)>0
        diffPredict = diffPredict + step;
    end
    if bitget(delta(i),2)>0
        diffPredict = diffPredict + bitshift(step,-1);
    end
    if bitget(delta(i),1)>0
        diffPredict = diffPredict + bitshift(step,-2);
    end
    if bitget(delta(i),4)>0
        predictedSample = predictedSample-diffPredict;
        if predictedSample<-32768
            predictedSample = -32768;
        end
    else
        predictedSample = predictedSample+diffPredict;
        if predictedSample>32767
            predictedSample = 32767;
        end
    end
    index = index + indexTable(delta(i)+1);
    if index<0
        index = 0;
    end
    if index>88
        index = 88;
    end
    data(i) = predictedSample;
end
rightData = data;

delta = leftDelta;
data = zeros(m,1);
index = 0;
predictedSample = 0;
for i = 1:1:m,
    step = stepsizeTable(index+1);
    diffPredict = bitshift(step,-3);
    if bitget(delta(i),3)>0
        diffPredict = diffPredict + step;
    end
    if bitget(delta(i),2)>0
        diffPredict = diffPredict + bitshift(step,-1);
    end
    if bitget(delta(i),1)>0
        diffPredict = diffPredict + bitshift(step,-2);
    end
    if bitget(delta(i),4)>0
        predictedSample = predictedSample-diffPredict;
        if predictedSample<-32768
            predictedSample = -32768;
        end
    else
        predictedSample = predictedSample+diffPredict;
        if predictedSample>32767
            predictedSample = 32767;
        end
    end
    index = index + indexTable(delta(i)+1);
    if index<0
        index = 0;
    end
    if index>88
        index = 88;
    end
    data(i) = predictedSample;
end
leftData = data;

stereoData=[leftData,rightData];

soundsc(double(rightData))
