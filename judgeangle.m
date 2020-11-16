function [] = judgeangle(input)
if input(1) >= 160 | input(1) <= -160
    fprintf("theta 1 is out of range!\n")
end
if input(2) >= 125 | input(2) <= -125
    fprintf("theta 2 is out of range!\n")
end
if input(3) >= 135 | input(3) <= -135
    fprintf("theta 3 is out of range!\n")
end
if input(4) >= 140 | input(4) <= -140
    fprintf("theta 4 is out of range!\n")
end
if input(5) >= 100 | input(5) <= -100
    fprintf("theta 5 is out of range!\n")
end
if input(6) >= 260 | input(6) <= -260
    fprintf("theta 6 is out of range!\n")
end
fprintf("----------------------------------------------------------------\n")
end