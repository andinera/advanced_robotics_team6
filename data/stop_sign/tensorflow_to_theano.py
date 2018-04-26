from keras import backend as K
from keras.utils.conv_utils import convert_kernel
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D
from keras.layers import Activation, Dropout, Flatten, Dense


img_width = 640
img_height = 480
input_shape = (img_height, img_width, 3)
# Layers chosen based on passed model size
model = Sequential()
model.add(Conv2D(32, (3, 3), input_shape=input_shape))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Conv2D(32, (3, 3)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Conv2D(64, (3, 3)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Flatten())
model.add(Dense(64))
model.add(Activation('relu'))
model.add(Dropout(0.5))
model.add(Dense(1))
model.add(Activation('sigmoid'))

model.load_weights('stop_sign_3.h5')

for layer in model.layers:
  if layer.__class__.__name__ == 'Conv2D':
      print 'hello'
      w, b = layer.get_weights()
      w = convert_kernel(w)
      layer.set_weights([w, b])

model.save_weights('my_weights_theano.h5')
