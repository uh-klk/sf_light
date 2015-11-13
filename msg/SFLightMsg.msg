uint8                   mode            # mode to execute
uint8                   NONE = 0        # turn off the LED Display
uint8                   STATIC = 1      # turn on the LED Display with a specified color
uint8                   FLASH = 2       # flashing the LED Display between the specified color and black
uint8                   BREATH = 3      # fading the LED Display in and out of the specified color
uint8                   SWEEP = 8       # 1 led running in clockwise

float32                 frequency 	    # hz - blink/sec, round/sec
uint8                   priority 	      # priority between 0 to 100. Default priority is 50,
std_msgs/ColorRGBA      color           # A not being use
float32                 on_pct 	        # duration the light should be on per cycle in %
                                        # i.e full cycle = 1 
                                        #     half cycle = 0.5 
