#!/bin/bash
g++ -v main.cpp -o main $(pkg-config --cflags --libs opencv4)

