var comSock = new WebSocket("ws://localhost:8888/ws")
comSock.onopen = function(event){
  console.log("Socket opened");
  comSock.send("Hello!");
}
comSock.onmessage = function(event){
  console.log("Received data "+event.data)
}
comSock.onclose = function(event){
  console.log("Websocket disconnected")
  alert("Websocket disconnected")
}
