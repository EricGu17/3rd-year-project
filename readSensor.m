function [sValues, frame_front_binarized] = readSensor(self)

self.update_cameras();

frame_front = im2gray(self.frame_front);
frame_front_binarized = imbinarize(frame_front);

sensor_values = frame_front(end,:);
sensor_values = [sensor_values 127];

sValues = zeros(1,5);
sc=6; 

for i=1:5
    sValues(i) = mean(sensor_values((i-1)*sc+1 : i*sc));
    if(sValues(i)<50) %Threshold of pixel intensity
        sValues(i) = 1;
    else
        sValues(i) = 0;
    end
end

end