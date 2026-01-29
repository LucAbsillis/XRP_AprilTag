import commands2
import ntcore
import math
from subsystems.drivetrain import Drivetrain
import time

class DriveToTag(commands2.Command):
    def __init__(self, tagToFind: int, drive: Drivetrain) -> None:
        """ commands, first looks for specific april tag 
        if found calculates bearing and distance, turns robot towards target and drives distance minus 20 cm  (7.8 inch)
        :param tagToFind: april tag id
        :param drive:  The drivetrain subsystem on which this command will run
        """
        super().__init__()
        # XRP drive behaviour is tricky tweaking speeds helps 
        self.speed = 0.7
        self.turnspeed= 0.6
        self.drive = drive
        self.tag = tagToFind
        self.addRequirements(drive)
        # self.state = ["looking","initDrive","turning","driving","done"]
        self.currentState ="Looking" 

    def initialize(self) -> None:
        """Called when the command is initially scheduled."""
        print("Drive to TAG")
        self.drive.arcadeDrive(0, 0)
        self.drive.resetEncoders()
            # Get the default NetworkTables instance
        inst = ntcore.NetworkTableInstance.getDefault()

         # Start a NetworkTables 4 client and connect to the local simulator
        inst.startClient4("driveToTag")
        inst.setServer("localhost")
        self.apriltags = inst.getTable("apriltags")
        self.tagID=self.apriltags.getIntegerArrayTopic("tags").subscribe([])
        self.stopCommand=False
        self.drive.arcadeDrive(0, 0)
        self.drive.resetEncoders()
        self.updateTagFound()
        self.rotationCount=0
        self.rotAngle=20
        self.inchPerDegree = math.pi * 6.102 / 360
        self.updateTagFound()
        if self.tagFound:  
            self.currentState = "initDrive"
            self.initDrive()
        else:
            self.currentState = "looking"
    def execute(self) -> None:
        """Called every time the scheduler runs while the command is scheduled.""" 
        #TODO Make this non-blocking
        if self.currentState == "looking" :
            self.locateTag()
        elif self.currentState == "initDrive":
            self.initDrive()
        else:
            #self.celf.currentState = sameraDrive()
            self.commandDrive()

    def end(self, interrupted: bool) -> None:
        """Called once the command ends or is interrupted."""
        self.drive.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        # Compare distance travelled from start to desired distance
        return self.stopCommand
    
    # def getAverageTurningDistance(self) -> float:
    #     leftDistance = abs(self.drive.getLeftDistanceInch())
    #     rightDistance = abs(self.drive.getRightDistanceInch())
    #     return (leftDistance + rightDistance) / 2.0
    
    # get current tagID from network table return if tagToFind is visible
    def updateTagFound(self):
        self.tagvalue = self.tagID.get()
        self.tagFound = len(self.tagvalue) > 0 and self.tagvalue[0]==self.tag

    # looks for apriltag with tagToFind ID by performing 360 turn until found, commands stops if not found  
    def locateTag(self):
        print("looking")
        self.updateTagFound()
        if not(self.tagFound):
            if self.rotationCount > 360/self.rotAngle:
                print("Completed search turn")
                self.drive.arcadeDrive(0, 0)
                self.stopCommand=True
            else:
                if self.rotationCount == 0:
                        #start first rotation
                        print(f"rotation: {self.rotationCount}") 
                        self.rotationCount = self.rotationCount+1
                        self.drive.arcadeDrive(self.speed,self.turnspeed)# turn slow
                if self.self.drive.getAverageDistanceInch() >= self.inchPerDegree * self.rotAngle:
                    # end of turn segment stop rotating and check for vision
                    print("stopping rotation")
                    self.drive.arcadeDrive(0, 0)
                    time.sleep(1)
                    # need to wait for camera processing to catch up 
                    self.updateTagFound()
                    if not(self.tagFound):
                        # reset encoders condinue turn
                        self.drive.resetEncoders()
                        self.rotationCount = self.rotationCount+1
                        print(f"rotation: {self.rotationCount}")
                        self.drive.arcadeDrive(self.speed,self.turnspeed)# turn 
                    else:
                        self.currentState = "initDrive"
                        self.initDrive()
                else:
                    self.drive.arcadeDrive(self.speed,self.turnspeed)# turn   
        else:
            # init drive to tag
            self.currentState = "initDrive"
            self.initDrive()

    # initialize bearing and distance to tagToFind before d    
    def initDrive(self):
        # make sure tag is still visible
        self.updateTagFound()
        if self.tagFound:  
            print(f"Tag {self.tagFound} found")
            self.drive.arcadeDrive(0, 0)
            self.relativePosition=self.apriltags.getDoubleArrayTopic(f"aprilID_{self.tag}").subscribe([]) 
            self.currentState= ""
            self.commandDrive() 

    # called when tagToFind is visible, gets bearing and distance to tag, turns towards tag and drives distance - 20 cm
    def commandDrive(self):
        if self.currentState == "initDrive":
            position=self.relativePosition.get()
            self.distanceToTarget= (abs(position[0])-0.2) *39.3701 #substract 20 cm and convert meters to inches 
            self.bearing = position[1]
            if self.bearing > 5: # turn if more than 5 degrees 
                self.drive.resetEncoders()
                self.currentState = "turning"
                self.turn(self.bearing)
        elif self.currentState == "turning":
            self.turn(self.bearing)
        elif self.currentState == "driving":
            self.driveForward(self.distanceToTarget)

    # turn robot degrees
    def turn(self,degrees:float):
        print(f"Turning {degrees} degrees")
        if self.drive.getAverageDistanceInch() >= self.inchPerDegree * degrees:
            self.drive.arcadeDrive(0, 0)
            self.drive.resetEncoders()
            self.currentState = "driving"          
            self.driveForward(self.distanceToTarget)
        else:     
            if degrees >0:
                self.drive.arcadeDrive(self.speed,-self.turnspeed)# turn 
            else:
                self.drive.arcadeDrive(self.speed,self.turnspeed)  
                
    # drive robot forward until distance is reached
    def driveForward(self,distance:float):
        print(f"driving {distance} inches forward")
        if self.drive.getAverageDistanceInch() >= distance:
            self.drive.arcadeDrive(0, 0)
            print("Reached tag")
            self.stopCommand=True
            self.currentState="done"
        else:
            self.drive.arcadeDrive(self.speed,0)# drive forward  
            
    # alternative appoach 
    #  drives towards target by re-adjusting bearing and position, fails often when tag visibility is lost 
    def cameraDrive(self):
        position=self.apriltags.getDoubleArrayTopic(f"aprilID_{self.tag}").subscribe([]) 
        if abs(position[0]) < 0.2: 
            # stop if distance < 20 cm
            print ("Reached tag")
            self.drive.arcadeDrive(0, 0)           
            self.stopCommand=True
            self.currentState="done"
        if abs(position[1]) > 5:
            # turn to if distance is angle >5 degrees
            # TODO figure out angle value
            # TODO check directionc
            if position[1] > 0:
                print ("turning negative")
                self.drive.arcadeDrive(self.speed,-self.turnspeed)# turn slow
            else:
                print ("turning positive")
                self.drive.arcadeDrive(self.speed,self.turnspeed) # turn slow
        else:
            print("driving forward")
            self.drive.arcadeDrive(self.speed,0) 
        # recheck tag visibility and stop if lost 
        self.updateTagFound()
        if not(self.tagFound):
            print("Lost tag visibility")
            self.drive.arcadeDrive(0, 0)
            self.stopCommand=True