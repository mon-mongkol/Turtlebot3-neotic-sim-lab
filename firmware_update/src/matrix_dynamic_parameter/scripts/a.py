#!/usr/bin/env python2
import os 

# get robot model 
try:
    model_ = os.environ['model']
except:
    model_ = "uvc"

if model_ == 'uvc':
    print "hooyaa"

print(model_)