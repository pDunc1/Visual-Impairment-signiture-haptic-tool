imdata=imread('SigTest.jpeg'); % Import Jpeg image of trace
imshow(imdata) % Show jpeg image of trace

input = 20; % Chosen number of points to click and follow trace. 20 = number of clicks along trace
[xal,yal] = ginput(input);

Points = (input/ (input -1) )/ input; % No of points

xt = [-xal'./100;yal'./100]; % -xal for inverted image in x axis for Haply world. /100 for scaling
figure (1);
axis('equal')

plot(-xal./100,yal./100)

%Set at origin
Ax = gca;
Ax.XAxisLocation = 'origin'; 
Ax.YAxisLocation = 'origin';

% 1/points to form trace 
s=0:Points:1;

%10th order Bezier Function
Kn10=@(s) [(1.*(s.^0).*((1-s).^10)); (10.*(s.^1).*((1-s).^9)); (45.*(s.^2).*((1-s).^8)); ... 
            (120.*(s.^3).*((1-s).^7)); (210.*(s.^4).*((1-s).^6)); (252.*(s.^5).*((1-s).^5)); ...
            (210.*(s.^6).*((1-s).^4)); (120.*(s.^7).*((1-s).^3)); (45.*(s.^8).*((1-s).^2)); ...
            (10.*(s.^9).*((1-s).^1)); (1.*(s.^10).*((1-s).^0))];
        
X = Kn10(s); %10th order Bezier Function
P = xt/X; %Bezier coordinates

% calls bez2_10th order function
bez2_10thOrder

%Saves Bezier Coordinate points 'P' as csv file within same folder as processing code
save('BezierPoints.mat','P') 
FileData = load('BezierPoints.mat');
csvwrite('/Users/paulduncan/Library/Mobile Documents/com~apple~CloudDocs/UoR/Year 3/3rd Year Project/Haply/Haptic/Getting-Started-master/Getting-Started-master/Signature/GuidedExploration_10thOrder/BezierPoints10.csv', FileData.P);

% Modified from Dr William Harwins Code. University of Reading