#include <X11/Xlib.h>
#include <GL/glx.h>
#include <stdio.h>
int main(){
  Display *d = XOpenDisplay(NULL);
  if(!d){ printf("NO_DISPLAY\n"); return 1; }
  int attr[] = {GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None};
  XVisualInfo *vi = glXChooseVisual(d, 0, attr);
  if(!vi){ printf("NO_VISUAL\n"); return 2; }
  Window root = DefaultRootWindow(d);
  XSetWindowAttributes swa; swa.colormap = XCreateColormap(d, root, vi->visual, AllocNone); swa.event_mask = 0;
  Window w = XCreateWindow(d, root, 0,0, 64,64, 0, vi->depth, InputOutput, vi->visual, CWColormap|CWEventMask, &swa);
  GLXContext ctx = glXCreateContext(d, vi, NULL, GL_TRUE);
  if(!ctx){ printf("NO_CONTEXT\n"); return 3; }
  glXMakeCurrent(d, w, ctx);
  printf("GL=%s | %s\n", glGetString(GL_RENDERER), glGetString(GL_VERSION));
  return 0;
}
