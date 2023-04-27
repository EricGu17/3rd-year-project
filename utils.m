function [sValues, frame_front_binarized,frame_left_binarized,frame_right_binarized] = utils(self)

frame_front = rgb2gray(self.frame_front);
frame_left = rgb2gray(self.frame_left);
frame_right = rgb2gray(self.frame_right);

frame_front_binarized = imbinarize(frame_front);
frame_left_binarized = imbinarize(frame_left);
frame_right_binarized = imbinarize(frame_right);

sValues = zeros(1,5);
for i=1:5
    sValues(i) = mean(sensor_values((i-1)*6+1 : i*6));
    if(sValues(i)<50)
        sValues(i) = 1;
    else
        sValues(i) = 0;
    end
end

end