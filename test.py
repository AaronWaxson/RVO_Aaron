X = [[-0.5+1.0*i, 0.0] for i in range(7)]
print(X)


import matplotlib
import matplotlib.pyplot as pyplot
# figure = pyplot.figure()
# ax = figure.add_subplot(1, 1, 1)
# srec = matplotlib.patches.Rectangle(
#     (0, 0),
#     2, 2,
#     facecolor='red',
#     fill=True,
#     alpha=1)
# ax.add_patch(srec)
# ax.arrow(1, 1, 3, 3,
#          head_width=0.05, head_length=0.1, fc='red', ec='red')
a = [[-0.3, 2.5, 0.3],[-0.3, 1.5, 0.3]]
pyplot.plot([x[0] for x in a], [x[1] for x in a], '-')
pyplot.show()


# # input()
# fp = open("./bad.txt", 'w')
# index = 2
# fp.write("%d is bad.\n" % index)
# fp.write("%d is bad." % index)
# fp.close()
# a = [[-0.3, 2.5, 0.3],[-0.3, 1.5, 0.3], [3.3, 1.5, 0.3], [3.3, 3.5, 0.3]]
# print(min(a))
# def alpha(a):
#     a += 1
#     print(a)
# a = 6
# alpha(a)
# print(a)

# ws_model = dict()
# ws_model['circular_obstacles'] = [[-0.3, 1.5, 0.3],
#                                   [-0.3, 3.5, 0.3], [3.3, 1.5, 0.3], [3.3, 3.5, 0.3]]

# co = ws_model['circular_obstacles']
# print(co-ws_model['circular_obstacles'])
# for i in range(len(co)):
#     ws_model['circular_obstacles'][i][0] = co[i][0] + 0.2

# print(ws_model['circular_obstacles'])


# for i in range(7):
    # print(i)

# X = [[-0.5+1.0*i, 0.0]
#      for i in range(7)] + [[-0.5+1.0*i, 5.0] for i in range(7)]
# print(X)
# goal = [[7, 7], [6, 6]]
# V_max = [0.5+i*0.1 for i in range(len(x))]
# # print(V_max)
# for i in range(len(goal)):
#     diff_x = [goal[i][k]-x[i][k] for k in range(2)]
#     print(diff_x)
