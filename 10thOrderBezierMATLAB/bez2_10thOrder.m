%% Bezier curve 1
% Replicate Ozan's work
if ~ exist('s','var'); s=-.1:.01:1.1;end % s is normally betweeen 0 and 1
Kn10=@(s) [(1.*(s.^0).*((1-s).^10)); (10.*(s.^1).*((1-s).^9)); (45.*(s.^2).*((1-s).^8)); ... 
            (120.*(s.^3).*((1-s).^7)); (210.*(s.^4).*((1-s).^6)); (252.*(s.^5).*((1-s).^5)); ...
            (210.*(s.^6).*((1-s).^4)); (120.*(s.^7).*((1-s).^3)); (45.*(s.^8).*((1-s).^2)); ...
            (10.*(s.^9).*((1-s).^1)); (1.*(s.^10).*((1-s).^0))];


% Cubic control points P
if ~exist('P','var'); P=randn(2,4);end

x=P*Kn10(s); %points on a cubic bezier curve

% distfn=@(s) (P*Kn3(s))'*(P*Kn3(s));
% set at the moment to be the origin
distfn=@(s) (Kn10(s))'*P'*P*(Kn10(s));

res0=fminsearch(distfn,0);
res1=fminsearch(distfn,1);
if abs(res0-res1)<1E-3; disp('One solution found');end

%Produces Bezier Coordinates
nst0=P*Kn10(res0);
nst1=P*Kn10(res1);
figure(1)
plot(x(1,:),x(2,:),P(1,:),P(2,:),'o',[0 nst0(1)],[0,nst0(2)],[0 nst1(1)],[0,nst1(2)])
axis('equal')
% solution sensitive to starting position
% Modified from Dr William Harwins code University of Reading
