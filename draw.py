import turtle

def get_color(i):
  colors = ["#ff5744", "#41befc", "#52bf54", "#32bf54", "#12bf54"]
  return colors[i%len(colors)]


myPen = None
def draw(circles, path):
  global myPen
  myPen = turtle.Turtle()
  # turtle.resizemode(rmode='auto')
  myPen.hideturtle()
  #myPen.tracer(0)
  myPen.speed(0)
  
  window = turtle.Screen()
  window.bgcolor("#F0F0F0")

  for i, path in enumerate(path):
    draw_cross(path, "#888888")

  for i, (r, p) in enumerate(circles):
    draw_cross(p, "#52bf54")
    
    myPen.goto(p.x,p.y-r)
    myPen.pendown()
    myPen.circle(r)

    myPen.getscreen().update()

def draw_cross(location, color):
  global myPen
  myPen.color(color)
  myPen.penup()
  myPen.goto(location.x-5,location.y)
  myPen.pendown()
  myPen.goto(location.x+5,location.y)
  myPen.penup()
  myPen.goto(location.x,location.y-5)
  myPen.pendown()
  myPen.goto(location.x,location.y+5)
  myPen.penup()

  myPen.getscreen().update()