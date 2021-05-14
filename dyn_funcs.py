def f1():
    print('f1')
    return 'f1'

def f2():
    print('f2')

def f3():
    print('f3')

names = ['f1','f2','f3']
print(eval(names[0])())