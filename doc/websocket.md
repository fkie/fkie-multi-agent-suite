# Websocket communication

The [websockets](https://websockets.readthedocs.io/en/stable/) has been expanded with mechanisms such as **publish**/**subscribe** and **remove procedure calls**.
>Since we use threads, a version of websockets >= 12.0 is required.

To subscribe to data from a publisher, simply send, for example, `{ “sub”, [‘ros.daemon.ready’] }` to the WebSocket.
You will then receive the message `{ “uri”: ‘ros.daemon.ready’, “message”: JSONObject }` every time the data is published at the interface URI `ros.daemon.ready`.

Calling a service also requires an `id`. It is necessary so that the response can be assigned to the calling function.

The following table contains the specifications for messages exchanged via the WebSocket.

| Action                      | Message Format                               | Description                                                                              |
| --------------------------- | -------------------------------------------- | ---------------------------------------------------------------------------------------- |
| Publisher message           | `{ "uri": str, "message": JSONObject }`      | Publish messages to given topic (URI)                                                    |
| Subscribe to an URI         | `{ "sub", [str] }`                           | Receive all messages published to this URI                                               |
| Unsubscribe from an URI     | `{ "unsub", [str] }`                         | Stop receiving messages send to this URI                                                 |
| Remote procedure call (RPC) | `{ "uri": str, "id": number, "params": [] }` | id is unique request. params is a is list with parameters for the remote procedure call. |
| Reply for successful RPC    | `{ "id": number, "result": JSONObject }`     | id is unique request id.                                                                 |
| Reply with error while RPC  | `{ "id": number, "error": str }`             | id is unique request id.                                                                 |
