# Construct the regression neural network
# Construct the network
inputs = Input(shape=(224,224,3))
# Block 1
x = Conv2D(64, (3, 3), activation='relu', padding='same', name='block1_conv1')(inputs)
x = Conv2D(64, (3, 3), activation='relu', padding='same', name='block1_conv2')(x)
x = MaxPooling2D((2, 2), strides=(2, 2), name='block1_pool')(x)

# Block 2
x = Conv2D(128, (3, 3), activation='relu', padding='same', name='block2_conv1')(x)
x = Conv2D(128, (3, 3), activation='relu', padding='same', name='block2_conv2')(x)
x = MaxPooling2D((2, 2), strides=(2, 2), name='block2_pool')(x)

# Block 3
x = Conv2D(256, (3, 3), activation='relu', padding='same', name='block3_conv1')(x)
x = Conv2D(256, (3, 3), activation='relu', padding='same', name='block3_conv2')(x)
x = Conv2D(256, (3, 3), activation='relu', padding='same', name='block3_conv3')(x)
x = MaxPooling2D((2, 2), strides=(2, 2), name='block3_pool')(x)

# Block 4
x = Conv2D(512, (3, 3), activation='relu', padding='same', name='block4_conv1')(x)
x = Conv2D(512, (3, 3), activation='relu', padding='same', name='block4_conv2')(x)
x = Conv2D(512, (3, 3), activation='relu', padding='same', name='block4_conv3')(x)
x = MaxPooling2D((2, 2), strides=(2, 2), name='block4_pool')(x)

# Block 5
x = Conv2D(512, (3, 3), activation='relu', padding='same', name='block5_conv1')(x)
x = Conv2D(512, (3, 3), activation='relu', padding='same', name='block5_conv2')(x)
x = Conv2D(512, (3, 3), activation='relu', padding='same', name='block5_conv3')(x)
x = MaxPooling2D((2, 2), strides=(2, 2), name='block5_pool')(x)

x = Flatten()(x)

dimension   = Dense(512)(x)
dimension   = LeakyReLU(alpha=0.1)(dimension)
dimension   = Dropout(0.5)(dimension)
dimension   = Dense(3)(dimension)
dimension   = LeakyReLU(alpha=0.1, name='dimension')(dimension)

orientation = Dense(256)(x)
orientation = LeakyReLU(alpha=0.1)(orientation)
orientation = Dropout(0.5)(orientation)
orientation = Dense(BIN*2)(orientation)
orientation = LeakyReLU(alpha=0.1)(orientation)
orientation = Reshape((BIN,-1))(orientation)
orientation = Lambda(l2_normalize, name='orientation')(orientation)

confidence  = Dense(256)(x)
confidence  = LeakyReLU(alpha=0.1)(confidence)
confidence  = Dropout(0.5)(confidence)
confidence  = Dense(BIN, activation='softmax', name='confidence')(confidence)

model = Model(inputs, outputs=[dimension, orientation, confidence])