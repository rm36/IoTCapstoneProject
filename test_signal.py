import math
import random
from datetime import datetime
from collections import namedtuple
from draw import draw, draw_cross

Point = namedtuple('Point', ['x', 'y'])
DistanceAndPosition = namedtuple('DistanceAndPosition', ['distance', 'position'])


def format_float(f):
    if abs(f) < 0.001:
        return format(f, "^+02.6f")
    return format(f, "^+02.3f")


# Exact solution but inaccurate in practice.
def get_intersection(p1, r1, p2, r2, p3, r3):
  a = 2*p2.x - 2*p1.x
  b = 2*p2.y - 2*p1.y
  c = r1*r1 - r2*r2 - p1.x*p1.x + p2.x*p2.x - p1.y*p1.y + p2.y*p2.y
  d = 2*p3.x - 2*p2.x
  e = 2*p3.y - 2*p2.y
  f = r2*r2 - r3*r3 - p2.x*p2.x + p3.x*p3.x - p2.y*p2.y + p3.y*p3.y
  x = (c*e - f*b) / (e*a - b*d)
  y = (c*d - a*f) / (b*d - a*e)
  return Point(x,y)


# Multiple solutions, great in practice.
def get_two_intersections(p1, r1, p2, r2):
  d = math.sqrt((p1.x-p2.x)**2 + (p1.y - p2.y)**2)
  l = (r1**2 - r2**2 + d**2) / (2*d)
  if l**2 > r1**2 or d == 0:
    return None, None

  h = math.sqrt(r1**2 - l**2)
  sol1 = Point((l/d)*(p2.x-p1.x) + (h/d)*(p2.y-p1.y) + p1.x,
               (l/d)*(p2.y-p1.y) - (h/d)*(p2.x-p1.x) + p1.y)
  sol2 = Point((l/d)*(p2.x-p1.x) - (h/d)*(p2.y-p1.y) + p1.x,
               (l/d)*(p2.y-p1.y) + (h/d)*(p2.x-p1.x) + p1.y)
  return sol1, sol2


def get_distance(rssi):
  RSSI_AT_1M = -69
  N = 2
  distance = 10 ** ((RSSI_AT_1M - rssi)/(10 * N))
  return distance


def get_rssi(distance):
  RSSI_AT_1M = -69
  N = 2
  rssi = RSSI_AT_1M - 10 * N * math.log10(distance)
  return rssi


def get_delta_distance(p1, p2):
  dx1 = p1.x - p2.x
  dy1 = p1.y - p2.y
  return math.sqrt(dx1**2 + dy1**2)


def get_signal(position):
  FIXED_BEACON_POSITION = Point(x=30, y=40)
  distance = get_delta_distance(position, FIXED_BEACON_POSITION)

  # Simulate 1% Error
  return get_rssi(distance) * (1 + 0.01 * (random.random()-0.5))


pos = None
t = 0
def get_position():
  global pos
  global t
  if pos is None:
    pos = Point(x=0, y=0)
  
  # Update pos every time it's called
  t = t+1
  pos = Point(3*3-t*t, 2*t)

  return pos


def get_angle(p1, p2, p3):
  return math.atan2(p3.y - p1.y, p3.x - p1.x) - \
         math.atan2(p2.y - p1.y, p2.x - p1.x);


def are_colinear(p1, p2, p3):
  angle = get_angle(p1, p2, p3)
  if angle < 0.05 and angle > -0.05:
    return True
  if angle < math.pi + 0.05 and angle > math.pi - 0.05:
    return True
  return False


def should_add_data(dist_and_pos, cached_distance_and_position):
  if len(cached_distance_and_position) == 0:
    return True

  last_pos = cached_distance_and_position[-1].position
  delta = get_delta_distance(dist_and_pos.position, last_pos)
  if delta < 3:
    return False
  elif len(cached_distance_and_position) < 2:
    return True

  before_last_pos = cached_distance_and_position[-2].position
  return not are_colinear(before_last_pos, last_pos, dist_and_pos.position)


def get_average(points):
  avg = [0,0]
  for point in points:
    avg[0] += point.x
    avg[1] += point.y
  avg[0] /= len(points)
  avg[1] /= len(points)
  return Point(avg[0], avg[1])


def main():
  random.seed(datetime.now())

  cached_distance_and_position = []
  path = []
  for i in range(15):
    pos = get_position()
    path.append(pos)
    signal = get_signal(pos)
    print('signal:', signal)
    distance = get_distance(signal)
    pair=DistanceAndPosition(distance, pos)
    #print(i, pair, cached_distance_and_position)
    if should_add_data(pair, cached_distance_and_position):
      cached_distance_and_position.append(pair)
      all_intersection1 = []
      all_intersection2 = []
      for j in range(len(cached_distance_and_position) - 2):
        r1, p1 = cached_distance_and_position[j]
        r2, p2 = cached_distance_and_position[j+1]
        r3, p3 = cached_distance_and_position[j+2]
        
        # Exact solution but didn't work well
        #intersection = get_intersection(p1, r1, p2, r2, p3, r3)
        #all_intersection1.append(intersection)

        s1, s2 = get_two_intersections(p1, r1, p2, r2)
        if s1 is not None and s2 is not None:
          all_intersection1.append(s1)
          all_intersection2.append(s2)
  draw(cached_distance_and_position, path)
  
  p1 = get_average(all_intersection1)
  p2 = get_average(all_intersection2)
  
  print('\nAverage solutions:',p1, p2)

  draw_cross(p1, "#ff5744")
  draw_cross(p2, "#ff5744")

  # The actual beacon
  draw_cross(Point(30,40), "#000000")

  input()


if __name__ == '__main__':
  # Simple tests
  assert(are_colinear(Point(0, 1), Point(0,2), Point(0.1,7)))
  assert(not are_colinear(Point(0, 1), Point(0,2), Point(1,7)))

  main()