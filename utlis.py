def judgeangle(Tns):
    '''
    judge angle is vaild or not
    '''
    ret = [];count = 0
    for Tn in Tns:
        flag = False
        count += 1
        if Tn[0] >= 160 or Tn[0] <= -160:
            print('solution ', count, ' theta 1 is out of range !\n')
            print(Tn,'\n')
            flag = True
        elif Tn[1] >= 125 or Tn[1] <= -125:
            print('solution ', count, ' theta 2 is out of range !\n')
            print(Tn,'\n')
            flag = True
        elif Tn[2] >= 135 or Tn[2] <= -135:
            print('solution ', count, ' theta 3 is out of range !')
            print(Tn,'\n')
            flag = True
        elif Tn[3] >= 140 or Tn[3] <= -140:
            print('solution ', count, ' theta 4 is out of range !')
            print(Tn,'\n')
            flag = True
        elif Tn[4] >= 100 or Tn[4] <= -100:
            print('solution ', count, ' theta 5 is out of range !')
            print(Tn,'\n')
            flag = True
        elif Tn[5] >= 260 or Tn[5] <= -260:
            print('solution ', count, ' theta 6 is out of range !')
            print(Tn,'\n')
            flag = True

        elif flag == False:
            print('solution ', count, 'is vaild')
            print(Tn,'\n')
            ret.append(Tn)
    return ret

def collect_angle():
    '''
    let user input angle and check angle is vaild or not
    '''
    t1 = -1000; t2 = -1000; t3 = -1000; t4 = -1000; t5 = -1000; t6 = -1000
    while t1 >= 160 or t1 <= -160:
        t1 = int(input('pls input theta1: '))
    while t2 >= 160 or t2 <= -160:
        t2 = int(input('pls input theta2: '))
    while t3 >= 160 or t3 <= -160:
        t3 = int(input('pls input theta3: '))
    while t4 >= 160 or t4 <= -160:
        t4 = int(input('pls input theta4: '))
    while t5 >= 160 or t5 <= -160:
        t5 = int(input('pls input theta5: '))
    while t6 >= 160 or t6 <= -160:
        t6 = int(input('pls input theta6: '))

    return [t1, t2, t3, t4, t5, t6]