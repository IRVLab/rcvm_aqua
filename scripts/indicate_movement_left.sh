#!/bin/bash

rosservice call --wait /rcvm/indicate_movement "direction:
  x: 0.0
  y: 1.0
  z: 0.0"