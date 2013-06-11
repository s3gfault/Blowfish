#!/usr/bin/python


import sys, getopt,serial,signal,os,time

#file f = None

serport = ""
baudrate = 9600
filemode = "w"
outputfile = ""

def signal_handler(signal, frame):
        print 'You pressed Ctrl+C!'
	#ser.close()
	#f.close()
	print "File closed"
        sys.exit(0)

def cmd_line(argv):
	
	try:
	      opts, args = getopt.getopt(argv,"ab:hi:o:",["ifile=","ofile=","baudrate=","append="])
	except getopt.GetoptError:
	      print 'serial2file.py -i <inputfile> -o <outputfile> -b <baudrate> (-a)'
	      sys.exit(2)
	for opt, arg in opts:
	      if opt == '-h':
	     	 print 'serial2file.py -i <inputfile> -o <outputfile> -b <baudrate> (-a)'
        	 sys.exit(0)
	      elif opt in ("-i", "--ifile"):
		 global serport
	         serport = arg
      	      elif opt in ("-o", "--ofile"):
		 global outputfile 
	         outputfile = arg
	      elif opt in ("-b" , "--baudrate"):
		 global baudrate
		 baudrate = int(arg)
	      elif opt in ("-a","--append"):
		 global filemode
		 filemode = "a"
		
 	print "Serialport is: ", serport
	print "Output file is: ", outputfile
	print "Filemode is:", filemode
	print "Baudrate is: ", baudrate

def main(argv):

	signal.signal(signal.SIGINT, signal_handler)
	cmd_line(argv)
	
   	print
	print "Serialport is: ", serport
	print "Output file is: ", outputfile
	print "Filemode is:", filemode
	print "Baudrate is: ", baudrate
	
	raw_input("\n\nPress the enter key to start capturing.")

	fn = os.getcwd() + "/" + outputfile
	print fn
	f = open(fn,filemode)
	
	print "File opened is ", f.name
	print "Closed?: ", f.closed
	
	try:
		ser = serial.Serial(serport, baudrate,bytesize = 8, parity='N',stopbits=1, timeout=1)
		ser.open()		
   	except ser.SerialException as e:
        	print("could not open serial port '{}': {}".format(com_port, e))


    	# read response from serial port
#    	lines = []
    	while True:
      	 	line = ser.readline()
		 
	       	line = line.decode('ascii').rstrip()
		print "Writing: ", line
		f.write(line)
		f.write('\n')
        # wait for new data after each line
        	timeout = time.time() + 0.1
        	while not ser.inWaiting() and timeout > time.time():
	            pass
	        if not ser.inWaiting():
	            break 

    
	'''
	serstr = ""	
	while True:
		serstr = ser.readline()
		print "Writing: ", serstr	
		f.write(ser.readline())
		#f.write('\n')
	'''
	'''
	while True:
		str = raw_input("Enter your input: ");
		print "Received input is : ", str
		f.write(str)	
		f.write('\n')
	'''


	sys.exit(0)

if __name__ == "__main__":
	main(sys.argv[1:])


#	ser.open()

	
	
	

#	f.close()
