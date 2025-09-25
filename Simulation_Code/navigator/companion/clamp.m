function num = clamp(value,min_value,max_value)
%clamp Return a value within the given range
if value < min_value
    num = min_value;
elseif value > max_value
    num = max_value;
else
    num = value;
end
end

