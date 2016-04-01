#!/usr/bin/env python

import rospy
import sys

import alsaaudio
import wave

from manipulation.srv import *

import numpy as np
import psw
import gapi
import commands
# from ros_mary_tts.srv import *
import baxter_interface
from baxter_interface import CHECK_VERSION
from copy import deepcopy
from std_msgs.msg import (
    Empty,
    Bool,
)
from robot_functions import *

import threading
import thread

lock = threading.Lock()
#from baxter_demos.msg import obj_hypotheses,action

#------------------------------------------------------------------#
def speak(x):
    rospy.wait_for_service('ros_mary')
    try:
        print "waiting for service"
        add_two_ints = rospy.ServiceProxy('ros_mary',ros_mary)
        resp1 = add_two_ints(x)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#------------------------------------------------------------------#
speech = gapi.Speech('sp')
COUNTER = 0

global blueRequest
global greenRequest
global redRequest
blueRequest = 0
greenRequest = 0
redRequest = 0
global listen
listen = False
global calibrate
calibrate = False

if len(sys.argv)==2:
	if sys.argv[1] in gapi.languages.keys():
		speech.lang = gapi.languages[sys.argv[1]]
	elif sys.argv[1] in gapi.languages.values():
		speech.lang = sys.argv[1]

def handler(fileName):
    global speech,COUNTER
    global listen
    if listen == True:
        translator = gapi.Translator(speech.lang, 'en-uk')
        try:
            cfileName = psw.convert(fileName)
            phrase = speech.getText(cfileName)
            import os
            os.remove(fileName)
            os.remove(cfileName)

            all_phrases = {}
            count = 0
            max_key = 0
            max_value = 0
            dictionary_words = ['blue','green','red', 'one','two','three','four','3','too']
            #commands = {}
            #commands[0] = 'pick up the green object'
            #commands[1] = 'pick up the blue object below the green object'
            #commands[2] = 'pick up the blue object far from the green object'
            for j in phrase:
                all_phrases[count] = {}
                all_phrases[count]['text'] = str(j['text'])
                all_phrases[count]['score'] = 0

                for i in dictionary_words:
                	if i in all_phrases[count]['text']:
                		all_phrases[count]['score'] += 1

                if max_value < all_phrases[count]['score']:
                	max_value = all_phrases[count]['score']
                	max_key = count

                print 'phrase '+str(count+1)+' : ',str(j['text']),' score : ',str(all_phrases[count]['score'])

                count += 1

                phrase = all_phrases[max_key]['text']
                all_words = phrase.split(' ')
                words = phrase.split(' ')
                for i in range(len(words)):
                    words[i] = str(words[i])
                    all_words[i] = str(all_words[i])
                print all_words[i]

                print 'the phrase is:',phrase

                if 'blue' or 'do' or 'loo' or 'lu' or 'two' or '2' in words:
                    blueIndex = words.index('blue')
                    print words[blueIndex - 1]," blue sweets requested"
                    global blueRequest
                    blueNum = words[blueIndex - 1]
                    if 'one' in blueNum or '1' in  blueNum or blueNum == 'One' or 'wan' in blueNum:
                        blueRequest = 1
                    elif 'to' in blueNum or '2' in  blueNum or blueNum == 'two' or blueNum == 'Two':
                        blueRequest = 2
                    elif '3' in blueNum or  blueNum == 'three' or  blueNum == 'Three':
                        blueRequest = 3
                    else:
                        blueRequest = 0
                    # speak('Going to sleep!, sir.')
                    global listen
                    listen = False

                elif 'green' in words:
                    greenIndex = words.index('green')
                    print words[greenIndex - 1]," green sweets requested"
                    global greenRequest
                    greenNum = words[greenIndex - 1]
                    if 'one' in greenNum or '1' in greenNum or greenNum == 'One' or 'wan' in greenNum:
                        greenRequest = 1
                    elif 'to' in greenNum or '2' in greenNum or greenNum == 'two' or greenNum == 'Two' or greenNum == 'too':
                        greenRequest = 2
                    elif '3' in greenNum or  greenNum == 'three' or  greenNum == 'Three':
                        greenRequest = 3
                    else:
                        greenRequest = 0
                    # speak('Going to sleep!, sir.')
                    global listen
                    listen = False

            print "end of analysing speech"

        except Exception, e:
        	print "Unexpected error:", sys.exc_info()[0], e
        return True

def handle_voice(req):
    print "Received request"
    global blueRequest
    global greenRequest
    global redRequest
    global listen
    listen = True
    while listen == True:
        rospy.sleep(1)
    return VoiceRequestResponse([redRequest, greenRequest, blueRequest])

def main():
    print 'Speech to text recognition starting ..'
    # pub2 = rospy.Publisher('/action_from_voice', action, queue_size=1)
    #pub2 = rospy.Publisher('obj_manipulation_voice', obj_hypotheses, queue_size=1)
    rospy.init_node('Voice_recognition')

    t = rospy.Service('get_voice', VoiceRequest, handle_voice)

    rospy.spin()

def mic():
    lock.acquire()
    global calibrate
    if calibrate == False:
        mic = psw.Microphone()
        print 'sampling...'
        sample = np.array(mic.sample(200))
        print 'done'
    calibrate = True

    while True:
        mic.listen(handler, sample.mean(), sample.std())


if __name__ == '__main__':
    thread.start_new_thread(mic, ())
    main()
