count = 0;

for i = 1:V.Num
    V.Set(i).volume
    if V.Set(i).volume>=20
        count = count + 1;
    end
end