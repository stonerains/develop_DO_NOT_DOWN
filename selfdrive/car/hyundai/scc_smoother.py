import copy
import math

from common.numpy_fast import clip, interp
from cereal import car, log
from common.transformations import model
from selfdrive.car.hyundai.interface import ButtonType
from selfdrive.config import Conversions as CV, RADAR_TO_CAMERA
from selfdrive.car.hyundai.values import Buttons
from common.params import Params
from selfdrive.road_speed_limiter import road_speed_limiter_get_max_speed

# do not modify
V_CRUISE_DELTA_MI = 5 * CV.MPH_TO_KPH
V_CRUISE_DELTA_KM = 10
MIN_SET_SPEED = 30

ALIVE_COUNT = 10
WAIT_COUNT = 20

MAX_ACC_BUFFER_COUNT = 5
LIMIT_ACCEL = 8.
LIMIT_DECEL = 20.

ButtonType = car.CarState.ButtonEvent.Type
ButtonPrev = ButtonType.unknown
ButtonCnt = 0
LongPressed = False

class CruiseState:
  STOCK = 0
  SMOOTH = 1
  COUNT = 2

class SccSmoother:
  def __init__(self, accel_gain, decel_gain):

    self.accel_gain = accel_gain
    self.decel_gain = decel_gain

    self.last_cruise_buttons = Buttons.NONE
    self.target_speed = 0

    self.started_frame = 0
    self.accel_buf = []
    self.max_set_speed_buf = []
    self.max_set_speed = 0
    self.wait_timer = 0
    self.alive_timer = 0
    self.btn = Buttons.NONE

    self.alive_count = ALIVE_COUNT
    self.wait_count = WAIT_COUNT

    self.scc_smoother_enabled = Params().get('SccSmootherEnabled') == b'1'
    self.state = int(Params().get('SccSmootherState'))

  def reset(self):
    self.accel_buf = []
    self.max_set_speed_buf = []
    self.max_set_speed = 0
    self.wait_timer = 0
    self.alive_timer = 0
    self.btn = Buttons.NONE
    self.target_speed = 0

  def create_clu11(self, packer, alive_count, bus, clu11, button):
    values = copy.copy(clu11)
    values["CF_Clu_CruiseSwState"] = button
    values["CF_Clu_AliveCnt1"] = alive_count
    return packer.make_can_msg("CLU11", bus, values)

  def is_active(self, frame):
    return frame - self.started_frame <= self.alive_count + self.wait_count

  def dispatch_cancel_buttons(self, CC, CS):
    changed = False
    if self.last_cruise_buttons != CS.cruise_buttons:
      self.last_cruise_buttons = CS.cruise_buttons

      if not CS.cruiseState_enabled:
        if CS.cruise_buttons == Buttons.CANCEL:
          self.state += 1
          if self.state >= CruiseState.COUNT:
            self.state = 0

          Params().put('SccSmootherState', str(self.state))
          changed = True

    CC.sccSmoother.state = self.state

    return changed

  def cal_max_speed(self, frame, CC, CS):

    # TODO - Limits on curvature and road speed limit

    if self.max_set_speed == 0 or frame % 10 == 0:
      self.max_set_speed_buf.append(float(CC.cruiseOpMaxSpeed))
      if len(self.max_set_speed_buf) > 10:
        self.max_set_speed_buf.pop(0)

      self.max_set_speed = sum(self.max_set_speed_buf) / len(self.max_set_speed_buf)

  def update(self, enabled, can_sends, packer, CC, CS, frame, apply_accel, sm):

    if not self.scc_smoother_enabled:
      return

    send = False
    clu11_speed = CS.clu11["CF_Clu_Vanz"]

    self.cal_max_speed(frame, CC, CS)

    if self.dispatch_cancel_buttons(CC, CS):
      return send

    if self.state == CruiseState.STOCK or not CS.acc_mode or \
        not enabled or not CS.cruiseState_enabled or CS.cruiseState_speed < 1. or \
        CS.cruiseState_speed > 254 or CS.standstill or \
        CS.cruise_buttons != Buttons.NONE or \
        CS.brake_pressed:

      CC.sccSmoother.logMessage = '{:d},{:d},{:d},{:d},{:.1f},{:d},{:d},{:d},{:.1f}' \
        .format(int(CS.acc_mode), int(enabled), int(CS.cruiseState_enabled), int(CS.standstill), float(CS.cruiseState_speed),
                int(CS.cruise_buttons), int(CS.brake_pressed), int(CS.gas_pressed), float(clu11_speed))

      self.reset()
      self.wait_timer = ALIVE_COUNT + WAIT_COUNT
      return send

    current_set_speed = CS.cruiseState_speed * CV.MS_TO_KPH

    accel, override_acc = self.cal_acc(apply_accel, CS, clu11_speed, sm)

    if CS.gas_pressed:
      self.target_speed = clu11_speed
    else:
      self.target_speed = clu11_speed + accel

    self.target_speed = clip(self.target_speed, MIN_SET_SPEED, self.max_set_speed)

    CC.sccSmoother.logMessage = '{:.3f}/{:.3f}, {:.3f}, {:.1f}/{:.1f}, btn:{:d}' \
      .format(float(apply_accel*CV.MS_TO_KPH), float(override_acc), float(accel), float(self.target_speed), float(current_set_speed), int(self.btn))

    if self.wait_timer > 0:
      self.wait_timer -= 1
    else:

      if self.alive_timer == 0:
        self.btn = self.get_button(clu11_speed, current_set_speed)

      if self.btn != Buttons.NONE:
        can_sends.append(self.create_clu11(packer, self.alive_timer, CS.scc_bus, CS.clu11, self.btn))
        send = True

        if self.alive_timer == 0:
          self.started_frame = frame

        self.alive_timer += 1

        if self.alive_timer >= self.alive_count:
          self.alive_timer = 0
          self.wait_timer = self.wait_count
          self.btn = Buttons.NONE

    return send

  def get_button(self, clu11_speed, current_set_speed):

    error = self.target_speed - current_set_speed

    if abs(error) < 0.9:
      return Buttons.NONE

    return Buttons.RES_ACCEL if error > 0 else Buttons.SET_DECEL

  def get_lead(self, sm):

    radar = sm['radarState']
    model = sm['model']

    if radar.leadOne.status and radar.leadOne.modelProb > 0.3:
      return radar.leadOne

    try:
      radar = log.RadarState.LeadData.new_message()
      radar.leadOne.status = 1
      radar.leadOne.modelProb = model.lead.prob
      radar.leadOne.dRel = model.lead.dist - RADAR_TO_CAMERA
      radar.leadOne.vRel = model.lead.relVel
      radar.leadOne.yRel = model.lead.relY

      #radar.leadTwo.status = 1
      #radar.leadTwo.modelProb = model.leadFuture.prob
      #radar.leadTwo.dRel = model.leadFuture.dist - RADAR_TO_CAMERA
      #radar.leadTwo.vRel = model.leadFuture.relVel
      #radar.leadTwo.yRel = model.leadFuture.relY

      return radar.leadOne
    except:
      pass

    return None

  def cal_acc(self, apply_accel, CS, clu11_speed, sm):

    cruise_gap = clip(CS.cruise_gap, 1., 4.)
    #cruise_gap = interp(cruise_gap, [1, 2, 3, 4], [1., 1.5, 2.3, 2.8])

    override_acc = 0.
    v_ego = clu11_speed * CV.KPH_TO_MS
    op_accel = apply_accel * 3.5

    lead = self.get_lead(sm)
    if lead is None or lead.modelProb < 0.3:
      accel = op_accel
    else:

      if 3.5 < lead.dRel < -lead.vRel * (8. + cruise_gap):
        t = lead.dRel / lead.vRel
        acc = -(lead.vRel / t) * 4.57
        override_acc = acc

        if op_accel < 0:
          accel = (op_accel + acc) / 2.
        elif lead.vRel < -13.8:        #상대속도가 50킬로보다 클때 
          if CS.out.vEgo > 13.0:       #내 차 속도가 47킬로보다 클때 
            accel = acc / 3.5
          else:
            accel = acc / 3.0         
        elif lead.vRel < -11.1:        #상대속도가 40킬로보다 클때
          if CS.out.vEgo > 13.0:       #내 차 속도가 47킬로보다 클때
            accel = acc / 2.8
          else:
            accel = acc / 2.4  
        elif lead.vRel < -8.3:        #상대속도가 30킬로보다 클때
          if CS.out.vEgo > 13.0:       #내 차 속도가 47킬로보다 클때
            accel = acc / 2.6
          else:
            accel = acc / 2.3 
        elif lead.vRel < -5.5:        #상대속도가 20킬로보다 클때
          if CS.out.vEgo > 13.0:       #내 차 속도가 47킬로보다 클때
            accel = acc / 2.5
          else:
            accel = acc / 2.3
        elif lead.vRel < -2.7:        #상대속도가 20킬로보다 클때
          if CS.out.vEgo > 13.0:       #내 차 속도가 47킬로보다 클때
            accel = acc / 2.45
          else:
            accel = acc / 2.3             
        else:
          accel = acc / 2.2
 
 
      else:
        if lead.dRel > 15:             #차간 거리가 15m 보다 크고 
          if CS.out.vEgo < 15.0:       #내 차 속도가 54킬로보다 작을 때
            accel = op_accel * 1.5
          else:
            accel = op_accel * 1.39
        elif lead.dRel > 8:  
          if CS.out.vEgo < 8.3:
            accel = op_accel * 1.54
          else:
            accel = op_accel * 1.37
        else:
          accel = op_accel * 1.34


    if accel > 0.:
      accel *= self.accel_gain * 2.4
    else:
      accel *= self.decel_gain * 2.19

    return clip(accel, -LIMIT_DECEL, LIMIT_ACCEL), override_acc

  @staticmethod
  def update_cruise_buttons(controls, CS):

    car_set_speed = CS.cruiseState.speed * CV.MS_TO_KPH
    is_cruise_enabled = car_set_speed != 0 and car_set_speed != 255 and CS.cruiseState.enabled and controls.CP.enableCruise

    if is_cruise_enabled:
      if controls.CC.sccSmoother.state == CruiseState.STOCK:
        controls.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
      else:
        controls.v_cruise_kph = SccSmoother.update_v_cruise(controls.v_cruise_kph, CS.buttonEvents, controls.enabled, controls.is_metric)
    else:
      controls.v_cruise_kph = 0

    if controls.is_cruise_enabled != is_cruise_enabled:
      controls.is_cruise_enabled = is_cruise_enabled

      if controls.is_cruise_enabled:
        controls.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
      else:
        controls.v_cruise_kph = 0

    controls.v_cruise_kph = road_speed_limiter_get_max_speed(CS, controls.v_cruise_kph)
    controls.cruiseOpMaxSpeed = controls.v_cruise_kph

  @staticmethod
  def update_v_cruise(v_cruise_kph, buttonEvents, enabled, metric):

    global ButtonCnt, LongPressed, ButtonPrev
    if enabled:
      if ButtonCnt:
        ButtonCnt += 1
      for b in buttonEvents:
        if b.pressed and not ButtonCnt and (b.type == ButtonType.accelCruise or b.type == ButtonType.decelCruise):
          ButtonCnt = 1
          ButtonPrev = b.type
        elif not b.pressed and ButtonCnt:
          if not LongPressed and b.type == ButtonType.accelCruise:
            v_cruise_kph += 1 if metric else 1 * CV.MPH_TO_KPH
          elif not LongPressed and b.type == ButtonType.decelCruise:
            v_cruise_kph -= 1 if metric else 1 * CV.MPH_TO_KPH
          LongPressed = False
          ButtonCnt = 0
      if ButtonCnt > 70:
        LongPressed = True
        V_CRUISE_DELTA = V_CRUISE_DELTA_KM if metric else V_CRUISE_DELTA_MI
        if ButtonPrev == ButtonType.accelCruise:
          v_cruise_kph += V_CRUISE_DELTA - v_cruise_kph % V_CRUISE_DELTA
        elif ButtonPrev == ButtonType.decelCruise:
          v_cruise_kph -= V_CRUISE_DELTA - -v_cruise_kph % V_CRUISE_DELTA
        ButtonCnt %= 70
      v_cruise_kph = clip(v_cruise_kph, MIN_SET_SPEED, 144)

    return v_cruise_kph


