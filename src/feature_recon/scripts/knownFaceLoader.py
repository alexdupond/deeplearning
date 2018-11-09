#!/usr/bin/python3

import face_recognition
import glob
import numpy as np

path = "/home/quist/Pictures/*.jpg"

def init_encodings_lib(path):
    files = glob.glob(path)
    for index, person_name in enumerate(files):
        loaded_image = face_recognition.load_image_file(person_name)
        image_face_encoding = face_recognition.face_encodings(loaded_image)[0]
        file_name = '/home/quist/Pictures/human_' + str(index) + '.txt'
        #np.savetxt(file_name, 'Human_id:' + str(index) + '\n')
        #np.savetxt(file_name, 'Human_name:' + person_name[32:-4] + '\n')
        #np.savetxt(file_name, 'Encoding:')
        np.savetxt(file_name, image_face_encoding)

init_encodings_lib(path)