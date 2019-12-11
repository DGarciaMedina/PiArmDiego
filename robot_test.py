import piarm
import time

def openConnection(robot):
	if robot.alive:
		raise Exception("Robot is already switched on")
	robot.connect("/dev/ttyS0")
	if robot.alive:
		print("Success connecting to robot")
		return True
	else:
		print("Failed to connect to robot")
		return False


def closeConnection(robot):
	if not robot.alive:
		raise Exception("Robot is already switched off")
	robot.disconnect()
	if not robot.alive:
		print("Success disconnecting from robot")
		return True
	else:
		print("Failed to disconnect from robot")
		return False
		
def move_to_default_pos(robot):
	if robot.alive:
		DEFAULT = [500, 500, 500, 500, 500, 500]
		
		for ID in range(1, 7):
			robot.servoWrite(ID, int(DEFAULT[ID - 1]), 500)
			
		return True
	else:
		return False

def move_to_pos(robot, pos):
	if robot.alive:
				
		for ID in range(1, 7):
			robot.servoWrite(ID, int(pos[ID - 1]), 500)
			
		return True
	else:
		return False
		
def read_positions(robot):
	positions = [-1]*6
	if robot.alive:
		for ID in range(1, 7):
			print("Checking ID:", ID, "out of 7")
			response = robot.positionRead(ID)
			while 1:
				print(response)
				if type(response) == list:
					if len(response) >= 7:
						break
				# time.sleep(1)
				response = robot.positionRead(ID)
			print(response)
			pos = int.from_bytes(response[5]+response[6], byteorder='little')
			if pos > 999 or pos < 0:
				print("The position was invalid")
				return False
			else:
				print("The position received was",pos)
				positions[ID-1] = pos
		return positions	
	else:
		return False
            


def main():
	robot = piarm.PiArm()
	res = openConnection(robot)
	print(res)
	time.sleep(2)
	res = move_to_default_pos(robot)
	print(res)
	time.sleep(2)
	res = move_to_pos(robot,[800,500,500,700,500,500])
	print(res)
	time.sleep(2)
	res = move_to_pos(robot,[200,500,500,500,500,500])
	print(res)
	time.sleep(5)
	res = read_positions(robot)
	print("Reading pos:", res)
	res = closeConnection(robot)
	print(res)
	print("Successful test!") 
	
main()
