#include <arpa/inet.h>  // for structs
#include <stdio.h>
#include <stdlib.h>      // for exit()
#include <sys/socket.h>  // for TCP Connection

// create socket
int create_socket()
{
  int socket_1;

  printf("creating socket...\n");
  // AF_INET: IPv4, SOCK_STREAM: support interactive byte stream
  socket_1 = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

  return socket_1;
}

// bind socket
int bind_socket(int socket_1)
{
  int result = -1;
  int port = 8000;
  struct sockaddr_in server_addr = {0};

  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // 0.0.0.0
  server_addr.sin_port = htons(port);

  printf("binding socket...\n");
  result = bind(socket_1, (struct sockaddr *)&server_addr, sizeof(server_addr));

  return result;
}

int main()
{
  int socket_1 = create_socket();

  struct sockaddr_in client_addr;
  int len;

  // bind socket
  if (bind_socket(socket_1) < 0) {
    printf("cannot bind socket\n");
    exit(1);
  }

  // listen port
  if (listen(socket_1, 1) < 0) {
    printf("cannot listen port\n");
    exit(1);
  }

  // connect w/ clients
  printf("connecting clients");
  for (;;) {
    len = sizeof(client_addr);
    printf(".");

    if (accept(socket_1, (struct sockaddr *)&client_addr, &len) == -1) {
      printf("cannot connect with clients\n");
    } else {
      printf("connected with %s\n", inet_ntoa(client_addr.sin_addr));
    }
  }

  return 0;
}