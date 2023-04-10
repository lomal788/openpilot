# This is the work of JPR. "I love the Korean Cars!"
import copy
from cereal import car
from common.params import Params
from common.numpy_fast import clip, interp
from selfdrive.config import Conversions as CV
from common.realtime import DT_CTRL
from selfdrive.car.hyundai.values import CHECKSUM, LEGACY_SAFETY_MODE_CAR
import crcmod
hyundai_checksum = crcmod.mkCrcFun(0x11D, initCrc=0xFD, rev=False, xorOut=0xdf)

###### SPAS ###### - JPR
STEER_ANG_MAX = 360 # SPAS Max Angle
ANGLE_DELTA_BP = [0., 10., 20.]
ANGLE_DELTA_V = [1.19, 1.14, 1.09]    # windup limit
ANGLE_DELTA_VU = [1.29, 1.19, 1.14]   # unwind limit
TQ = 290 # = TQ / 100 = NM is unit of measure for wheel.
SPAS_SWITCH = 30 * CV.MPH_TO_MS # MPH - lowered Bc of model and overlearn steerRatio
STEER_MAX_OFFSET = 105 # How far from MAX LKAS torque to engage Dynamic SPAS when under 60mph.
###### SPAS #######

EventName = car.CarEvent.EventName

class SpasRspaController:
  def __init__(self):
    self.last_apply_angle = 0.0
    self.en_spas = 2
    self.mdps11_stat_last = 0
    self.lkas_active = False
    self.spas_active = False
    self.dynamicSpas = Params().get_bool('DynamicSpas')
    self.ratelimit = 2.3 # Starting point - JPR
    self.rate = 0
    self.lastSteeringAngleDeg = 0
    self.cut_timer = 0
    self.SteeringTempUnavailable = False
    self.ens_rspa = 0
    self.spas_mode_sequence = 2 if LEGACY_SAFETY_MODE_CAR else 1
    self.spas_mode_sequence = 2
  
  @staticmethod
  def create_rspa11(packer, frame, en_rspa, bus, enabled, setspeed, stopping, gaspressed):
    idx = int(frame / 2)
    values = {
      "CF_RSPA_State": en_rspa, # Match like SPAS state logic. - JPR
      "CF_RSPA_Act": 1 if en_rspa == (4 or 5) else 0, # RSPA Active. - JPR
      "CF_RSPA_DecCmd": 2 if enabled and stopping and not gaspressed else 0, # Are we stopping? - JPR
      "CF_RSPA_Trgt_Spd": 0.5, # 0.5kph as a test for now as a slow roll :) setspeed, Maybe can null message and use SCC? 2019+ non legacy seems to want both messages there...
      "CF_RSPA_StopReq": 1 if enabled and stopping and not gaspressed else 0, # Are we stopping? - JPR
      "CR_RSPA_EPB_Req": 0, # Electronic Parking Brake - JPR
      "CF_RSPA_ACC_ACT": 0, # 0 low speed? 1 High speed?
      "CF_RSPA_AliveCounter": idx % 0x10, # Happens to be same as SCC11! :) - JPR
      "CF_RSPA_CRC": 0,
    }
    # Handle RSPA CRC :) - JPR
    dat = packer.make_can_msg("RSPA11", 0, values)[2]
    dat = dat[:6] + dat[7:8]
    values["CF_RSPA_CRC"] = hyundai_checksum(dat)
    return packer.make_can_msg("RSPA11", bus, values)

  def create_spas11(packer, car_fingerprint, frame, en_spas, apply_steer, bus, spas_mode_sequence):
    values = {
      "CF_Spas_Stat": en_spas,
      "CF_Spas_TestMode": 1, # Maybe if set to 1 will ignore VS... needs testing.
      "CR_Spas_StrAngCmd": apply_steer,
      "CF_Spas_BeepAlarm": 0,
      "CF_Spas_Mode_Seq": spas_mode_sequence, # 2 if LEGACY_SAFETY_MODE_CAR else 1,
      "CF_Spas_AliveCnt": frame % 0x200, 
      "CF_Spas_Chksum": 0,
      "CF_Spas_PasVol": 0,
    }
    dat = packer.make_can_msg("SPAS11", 0, values)[2]
    # if car_fingerprint in CHECKSUM["crc8"]:
    #   dat = dat[:6]
    #   values["CF_Spas_Chksum"] = hyundai_checksum(dat)
    # else:
    #   values["CF_Spas_Chksum"] = sum(dat[:6]) % 256
    values["CF_Spas_Chksum"] = sum(dat[:6]) % 256
    return packer.make_can_msg("SPAS11", bus, values)

  def create_pas11(packer, bus): # SPAS Screen Prompts. - JPR
    values = {
    "CF_Gway_PASDisplayFLH": 0,
    "CF_Gway_PASDisplayFRH": 0,
    "CF_Gway_PASRsound": 0,
    "CF_Gway_PASDisplayFCTR": 0, 
    "CF_Gway_PASDisplayRCTR": 0,
    "CF_Gway_PASFsound": 0,
    "CF_Gway_PASDisplayRLH": 0,
    "CF_Gway_PASDisplayRRH": 0,
    "CF_Gway_PASCheckSound": 0,
    "CF_Gway_PASSystemOn": 0,
    "CF_Gway_PASOption": 0,
    "CF_Gway_PASDistance": 0,
    }
    #return packer.make_can_msg("SPAS12", bus, values) # When we want to do somthing with the SPAS prompts. - JPR
    return packer.make_can_msg("PAS11", bus, values)

  def create_spas12(packer, bus): # SPAS Screen Prompts. - JPR
    values = {
    "CF_Spas_HMI_Stat": 0,
    "CF_Spas_Disp": 0,
    "CF_Spas_FIL_Ind": 0,
    "CF_Spas_FIR_Ind": 0, 
    "CF_Spas_FOL_Ind": 0,
    "CF_Spas_FOR_Ind": 0,
    "CF_Spas_VolDown": 0,
    "CF_Spas_RIL_Ind": 0,
    "CF_Spas_RIR_Ind": 0,
    "CF_Spas_FLS_Alarm": 0,
    "CF_Spas_ROL_Ind": 0,
    "CF_Spas_ROR_Ind": 0,
    "CF_Spas_FCS_Alarm": 0,
    "CF_Spas_FI_Ind": 0,
    "CF_Spas_RI_Ind": 0,
    "CF_Spas_FRS_Alarm": 0,
    "CF_Spas_FR_Alarm": 0,
    "CF_Spas_RR_Alarm": 0,
    "CF_Spas_BEEP_Alarm": 0,
    "CF_Spas_StatAlarm": 0,
    "CF_Spas_RLS_Alarm": 0,
    "CF_Spas_RCS_Alarm": 0,
    "CF_Spas_RRS_Alarm": 0,
    }
    #return packer.make_can_msg("SPAS12", bus, values) # When we want to do somthing with the SPAS prompts. - JPR
    return [1268, 0, b"\x00\x00\x00\x00\x00\x00\x00\x00", bus]
  
  def create_ems_366(packer, ems_366, enabled):
    values = copy.copy(ems_366)
    if enabled:
      values["VS"] = 1
    return packer.make_can_msg("EMS_366", 1, values)

  def create_ems11(packer, ems11, enabled):
    values = copy.copy(ems11)
    if enabled:
      values["VS"] = 1
    return packer.make_can_msg("EMS11", 1, values)

  def create_eems11(packer, eems11, enabled):
    #values = copy.copy(eems11)
    if enabled:
      #values["Accel_Pedal_Pos"] = 1
      #values["CR_Vcu_AccPedDep_Pos"] = 1
      values = {
        "Brake_Pedal_Pos": 0,
        "IG_Reactive_Stat": 0,
        "Gear_Change": 0,
        "Cruise_Limit_Status": 0,
        "Cruise_Limit_Target": 0,
        "Accel_Pedal_Pos": 0,
        "CR_Vcu_AccPedDep_Pos": 0,
      }
    return packer.make_can_msg("E_EMS11", 1, values)

  def create_clu11(packer, clu11, enabled):
    values = copy.copy(clu11)
    if enabled:
      values["CF_Clu_Vanz"] = 1
    return packer.make_can_msg("CLU11", 1, values)

  def inject_events(self, events):
    if self.SteeringTempUnavailable:
      events.add(EventName.steerTempUnavailable)

  def RSPA_Controller(self, c, CS, frame, packer, can_sends, set_speed, stopping):
    if CS.rspa_enabled:
      if (frame % 1) == 0: # RSPA11 at 100hz. - JPR
        can_sends.append(SpasRspaController.create_rspa11(packer, frame, self.en_rspa, CS.mdps_bus, c.active, set_speed, stopping, CS.out.gasPressed))

  def SPAS_Controller(self, c, CS, actuators, frame, maxTQ, packer, car_fingerprint, emsType, apply_steer, turnsignalcut, can_sends):
    self.packer = packer
    self.car_fingerprint = car_fingerprint
    if CS.spas_enabled:
      # Keep Track of SPAS State, Steering wheel rate, and other metrics. - JPR
      self.rate = abs(CS.out.steeringAngleDeg - self.lastSteeringAngleDeg)
      apply_angle = clip(actuators.steeringAngleDeg, -1*(STEER_ANG_MAX), STEER_ANG_MAX)
      apply_diff = abs(apply_angle - CS.out.steeringAngleDeg)
      spas_active = c.active and CS.out.vEgo < 26.82 and (CS.out.vEgo < SPAS_SWITCH or apply_diff > 3.2 and self.dynamicSpas and not CS.out.steeringPressed or abs(apply_angle) > 3. and self.spas_active or maxTQ - STEER_MAX_OFFSET < apply_steer and self.dynamicSpas)      
      if (frame % 2) == 0: # Run this at same speed as the SPAS11 message BC thats how fast the steering updates. - JPR
        if CS.mdps11_stat == 5 and apply_diff > 1.75: # Rate limit for when steering angle is not apply_angle or "engage" rate. - JPR
          self.ratelimit += 0.03 # Increase each cycle - JPR
          rate_limit = max(self.ratelimit, 10) # Make sure not to go past +-10 on rate - JPR
          #print("apply_diff is greater than 1.5 : rate limit :", rate_limit)
          apply_angle = clip(apply_angle, CS.out.steeringAngleDeg - rate_limit, CS.out.steeringAngleDeg + rate_limit)
        elif CS.mdps11_stat == 5: # Normal Operation Rate Limiter. - JPR
          self.ratelimit = 2.3 # Reset it back - JPR
          if self.last_apply_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_apply_angle):
            rate_limit = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_V)
          else:
            rate_limit = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_VU)
          apply_angle = clip(apply_angle, self.last_apply_angle - rate_limit, self.last_apply_angle + rate_limit)
        else:
          apply_angle = CS.mdps11_strang

      if (CS.out.steeringPressedSPAS or self.rate > 1.2): # Reset SPAS cut timer if steeringPressedSPAS is True or if the steering wheel is moving with intent. - JPR
        self.cut_timer = 0
        spas_active = False
        
      if CS.out.steeringPressedSPAS or self.cut_timer <= 100:# Keep SPAS cut for 80 cycles after steering pressed to prevent unintentional fighting. - JPR
        spas_active = False
        self.cut_timer += 1
    
      if turnsignalcut:
        spas_active = False

      self.last_apply_angle = apply_angle

      ############### SPAS STATES ############## - JPR
      # State 1 : Start
      # State 2 : New Request
      # State 3 : Ready to Assist(Steer)
      # State 4 : Hand Shake between OpenPilot and MDPS ECU
      # State 5 : Assisting (Steering)
      # State 6 : Failed to Assist (Steer)
      # State 7 : Cancel
      # State 8 : Failed to get ready to Assist (Steer)
      # ---------------------------------------------------
      if spas_active and (CS.mdps11_stat == 4 or CS.mdps11_stat == 5 or CS.mdps11_stat == 3): # Spoof Speed on mdps11_stat 3, 4 and 5. - JPR
        spas_active_stat = True
      else:
        spas_active_stat = False
          
      if emsType == 1:
        can_sends.append(SpasRspaController.create_ems_366(self.packer, CS.ems_366, spas_active_stat))
        if Params().get_bool('SPASDebug'):
          print("EMS_366")
      elif emsType == 2:
        # can_sends.append(SpasRspaController.create_ems11(self.packer, CS.ems11, spas_active_stat))
        if Params().get_bool('SPASDebug'):
          print("EMS_11")
      elif emsType == 3:
        can_sends.append(SpasRspaController.create_eems11(self.packer, CS.eems11, spas_active_stat))
        if Params().get_bool('SPASDebug'):
          print("E_EMS11")
      #elif emsType == 4:
        #can_sends.append(SpasRspaController.create_clu11(self.packer, CS.clu11, spas_active_stat))
        #if Params().get_bool('SPASDebug'):
          #print("CLU11")
      #elif emsType == 0:
        #print("Please add a car parameter called ret.emsType = (your EMS type) in interface.py : EMS_366 = 1 : EMS_11 = 2 : E_EMS11 = 3")

      if (frame % 2) == 0:
        ####### !!!! DO NOT MODIFY SPAS STATE MACHINE - JPR !!!! #######
        if CS.mdps11_stat == 7 and not self.mdps11_stat_last == 7:
          self.en_spas = 7 # Acknowledge that MDPS is in State 7 - JPR

        if CS.mdps11_stat == 7 and self.mdps11_stat_last == 7:
          self.en_spas = 3 # Tell MDPS to get ready for next steer. - JPR

        if CS.mdps11_stat == 2 and spas_active:
          self.en_spas = 3 # Switch to State 3, and get Ready to Assist(Steer). - JPR

        if CS.mdps11_stat == 3 and spas_active:
          self.en_spas = 4 # Handshake Between MDPS and OpenPilot. - JPR

        if CS.mdps11_stat == 4: 
          self.en_spas = 5 # Ask MDPS to Steer using Angle. - JPR

        if CS.mdps11_stat == 5 and not spas_active:
          self.en_spas = 7 # Disengage/Cancel SPAS - JPR

        if CS.mdps11_stat == 6:
          self.en_spas = 2 # Failed to Assist and Steer, Set state back to 2 for a new request. - JPR

        if CS.mdps11_stat == 8:
          self.en_spas = 2 #MDPS ECU Fails to get into state 3 and ready for state 5. - JPR
      
        if CS.mdps11_stat == 6 or CS.mdps11_stat == 8: # Monitor MDPS SPAS error states and send them to inject_events. - JPR
          self.SteeringTempUnavailable = True
        else:
          self.SteeringTempUnavailable = False

        if not spas_active:
          apply_angle = CS.mdps11_strang if self.spas_mode_sequence == 2 else CS.sas11_angle

        #can_sends.append(SpasRspaController.create_spas11(self.packer, self.car_fingerprint, (frame // 2), self.en_spas, apply_angle, CS.mdps_bus, self.spas_mode_sequence))
      
      SpasRspaController.screen_controller(self, CS, can_sends, frame) # Access SPAS12 message controller for screen Prompts. - JPR
      
      if Params().get_bool('SPASDebug'): # SPAS debugging - JPR
        print("MDPS SPAS State: ", CS.mdps11_stat) # SPAS STATE DEBUG
        print("OP SPAS State: ", self.en_spas) # OpenPilot Ask MDPS to switch to state.
        print("spas_active:", spas_active)
        print("apply angle:", apply_angle)
        print("driver torque:", CS.out.steeringWheelTorque)
        #print("frame:", frame,'/',frame //2, '/' (frame //2) % 0x200)

      self.mdps11_stat_last = CS.mdps11_stat
      self.spas_active = spas_active
      self.lastSteeringAngleDeg = CS.out.steeringAngleDeg

  def screen_controller(self, CS, can_sends, frame):
    # SPAS12 20Hz
      if (frame % 5) == 0:
        can_sends.append(SpasRspaController.create_spas12(self.packer, CS.mdps_bus))
        #can_sends.append(SpasRspaController.create_pas11(self.packer, CS.mdps_bus))

  #def park_assist_system(self): ultrasonic radar sensors PAS. Will continue when I get bumper and PAS fixed. LOL

  #def distance_traveled(self, CS):
  #  self.VS = CS.out.vEgo # M/s
  #  self.time = 1 / DT_CTRL # Time
  #  if self.rising_edge == 1:
  #    self.time_passed = 1 / DT_CTRL # Time Passed
  #    self.distance_traveled = self.time_passed * self.VS
  #  else:
  #    self.time_passed = 0


  #def search_for_spot(self, CS):
  #  self.pas_fr_dif = self.pas_fr - self.pas_fr_last
  #  if CS.out.rightBlinker: # Search Right Side
  #    if self.pas_fr_dif > 2: # Edge detection 
  #      if self.falling_edge == 0: # which came first?
  #        self.rising_edge = 1
  #      elif self.falling_edge == 1:
  #        self.rising_edge = 2
  #    elif self.pas_fr_dif > -2: # Edge detection
  #      if self.rising_edge == 0: # which came first? 
  #        self.falling_edge = 1
  #      elif self.rising_edge == 1:
  #        self.falling_edge = 2

    #elif CS.out.leftBlinker: # Search Left Side
      #if self.pas_fl >:


  #def perpedicular_park(self):

  #def parallel_park(self):
  
  #def update(self):  # TODO move RSPA and SPAS to be called here
