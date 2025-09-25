function EnergyMap = GetMap_Object_new(x,y,Blocker,Target,Blocker_a,Blocker_b,Blocker_c,m2,n2,a,b,c,precision,ncf)
%GetMap Get an Energy Map based on SIFM
EnergyMap = nan(x,y);

parfor i = 1:x
    for j = 1:y
        pos = [i-1,j-1];

        %% decide the physical place that can't walk in;
        if isequal(pos,Blocker)
            EnergyMap(i,j) = inf;
            continue;
        end

        if pos(1) >= (Blocker(1)-0.5*Blocker_b) && pos(1) <= (Blocker(1)+0.5*Blocker_b)
            if pos(2) >= (Blocker(2)-0.5*Blocker_a) && pos(2) <= (Blocker(2)+0.5*Blocker_a)
                EnergyMap(i,j) = inf;
                continue;
            end
        end

        %% calculate the influence of myfield;
        if isequal(pos,Target)
            myfield = n2;
        else
            MyOri = acosd(sum((pos-Target).*[0,-1])/(norm(pos-Target)))+180;
            [CosB,SinB] = GetTriangle(Blocker,pos,MyOri);
            
            if CosB < 0
                fB = 0;
            else
                fB = CosB;
            end
            
            myfield = m2 * fB + n2 + c * (a*b)/sqrt((a * CosB)^2 + (b * SinB)^2);

        end

        %% calculate the influence of objectfield;
        d = norm(Blocker - pos) * precision;
        d1 = Blocker_a*precision*d/(2*abs(Blocker(2)-j)* precision);
        d2 = Blocker_b*precision*d/(2*abs(Blocker(1)-i)* precision);

        if (abs(Blocker(1)-i)/abs(Blocker(2)-j)) <= Blocker_b/Blocker_a % decide when use d1, when use d2;
            d_inside = d1;
        else
            d_inside = d2;
        end

        objectfield = Blocker_c * d_inside;

        %% get the energymap, combining both influence;
        
        EnergyMap(i,j) = (myfield * objectfield)/d^ncf;
        
    end
end

