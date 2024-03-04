# face detection with mtcnn on a photograph
from matplotlib import pyplot
from matplotlib.patches import Rectangle
from matplotlib.patches import Circle
from mtcnn.mtcnn import MTCNN
import cv2

# draw an image with detected objects
def draw_image_with_boxes(filename, result_list):
    # load the image
    data = pyplot.imread(filename)
    # plot the image
    pyplot.imshow(data)
    # get the context for drawing boxes
    ax = pyplot.gca()
    # plot each box
    for result in result_list:
        # get coordinates
        x, y, width, height = result['box']
        # create the shape
        rect = Rectangle((x, y), width, height, fill=False, color='red')
        # draw the box
        ax.add_patch(rect)
        # draw the dots
        for key, value in result['keypoints'].items():
            # create and draw dot
            dot = Circle(value, radius=2, color='red')
            ax.add_patch(dot)
    # show the plot
    pyplot.show()

# draw each face separately
def save_biggest_face_image(filename, result_list):
    # load the image
    # data = pyplot.imread(filename)
    data = cv2.imread(filename)
    # plot each face as a subplot
    maxlen = -1
    for i in range(len(result_list)):
        # get coordinates
        x1, y1, width, height = result_list[i]['box']
        if width > height:
            length = width 
        else:
            length = height
        x2, y2 = x1 + length, y1 + length
        # # define subplot
        # pyplot.subplot(1, len(result_list), i+1)
        # pyplot.axis('off')
        # # plot face
        # pyplot.imshow(data[y1:y2, x1:x2])
        d = data[y1:y2, x1:x2]
        if length > 100 and length > maxlen:
            maxlen = length
            md = d
        # cv2.imwrite('/Users/komatsu/Desktop/'+str(i)+'.png', d)

    # show the plot
    # pyplot.show()
    if maxlen > 0:
        cv2.imwrite('result.jpg', md)



#filename = 'test.png'
filename = 'test2.png'
# load image from file
pixels = cv2.cvtColor(cv2.imread(filename), cv2.COLOR_BGR2RGB)
# create the detector, using default weights
detector = MTCNN()
# detect faces in the image
faces = detector.detect_faces(pixels)
# display faces on the original image
draw_image_with_boxes(filename, faces)
#save_biggest_face_image(filename, faces)