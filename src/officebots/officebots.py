#!/usr/bin/env python3

import logging
logger = logging.getLogger(__name__)
import asyncio
import websockets
import json

class Robot:

    GAME_PORT=6970

    # Maximum time given to the game to acknowledge the
    # commands
    TIMEOUT=2 #s

    OK = "OK"
    ERROR = "EE"

    TIMEOUT_ERROR = [ERROR, "timeout"]
    CMD_ID_ERROR = [ERROR, "server response lost! (likely network issue)"]

    def __init__(self):
        self._event_loop = None

        self._cmd_id = 1

        self._msgs_to_game = asyncio.Queue()

        self._responses_from_game = asyncio.Queue()

        self._last_response = asyncio.Queue(maxsize=1)

    def stop(self):
        self._event_loop.stop()

    async def run(self):
        logger.warning("You need to override Robot.run!")
        return

    async def on_robot_update(self, data):
        """Called everytime new data is received from the robot. Override
           this method to process the data.
        """
        logger.debug(f"Received game-initiated msg: {data}")

    def start(self):
        self._event_loop = asyncio.get_event_loop()

        self._event_loop.set_exception_handler(self._handle_exception)

        self._event_loop.run_until_complete(
                        websockets.serve(self._handler, "localhost", self.GAME_PORT)
                        )

        logger.info("Started OfficeBots Python API")
        logger.info("Waiting for the game to connect on localhost:%s"% self.GAME_PORT)
        logger.info("Press the blue icon in the game to connect")

        try:
            self._event_loop.run_until_complete(self.run())
        except RuntimeError as re: # Event loop stopped before Future completed
            logger.error("Robot controller interrupted due to game disconnection")
            logger.error("Exception: %s" % str(re))

    async def execute(self, cmd):

        self.last_reponse = None
        if len(cmd) == 2: # no params? add an empty param list
            cmd.append([])
        self._msgs_to_game.put_nowait((self._cmd_id, cmd))
        self._cmd_id += 1

        #logger.debug("Waiting for response...")
        return await self._last_response.get()


    async def _send_cmd(self, websocket, path):
        while True:
            msg = await self._msgs_to_game.get()
            cmd_id, cmd = msg
            logger.debug("Sending to server %s" % str(cmd))
            await websocket.send(json.dumps(msg))
            try:
                response_id, response = await asyncio.wait_for(self._responses_from_game.get(), timeout=self.TIMEOUT)
            except TimeoutError:
                logger.error(f"Game timeout while waiting for response to cmd <%s>!" % cmd)
                self._last_response.put_nowait(self.TIMEOUT_ERROR)


            if response_id == cmd_id:
                logger.debug(f"Game responded: %s" % response)
                self._last_response.put_nowait(response)
            else:
                logger.error("Wrong command id! The game response to <%s> was lost somewhere!" % cmd)
                self._last_response.put_nowait(self.CMD_ID_ERROR)



        #break

    async def _recv_msgs(self, websocket, path):
        #async for msg in websocket:
        while True:
            msg = await websocket.recv()
            #logger.info(msg)
            if msg != "ack".encode():
                msg = json.loads(msg)

                cmd_id = msg[0]
                if cmd_id > 0: # this the response to a previous cmd
                    self._responses_from_game.put_nowait(msg)
                else: # cmd_id <= 0 -> msg initiated by the game
                    await self.on_robot_update(msg[1])

    async def _handler(self, websocket, path):
        logger.info("Game connected")

        consumer_task = asyncio.ensure_future(self._recv_msgs(websocket, path))
        producer_task = asyncio.ensure_future(self._send_cmd(websocket, path))

        done, pending = await asyncio.wait(
                [consumer_task, producer_task],
                return_when=asyncio.FIRST_COMPLETED,
        )

        for task in pending:
            task.cancel()

    def _handle_exception(self, loop, context):
        msg = context.get("exception", None)
        if msg:
            if hasattr(asyncio, 'exceptions') and type(msg) == asyncio.exceptions.TimeoutError:
                logger.error(f"The server did not answer our command! (timeout)")
                loop.stop()
            else:
                raise msg
        else:
            logger.error(f"Caught exception: {context['message']}")
            logger.info("Connection closed by the game (game stopped?). Exiting.")
            loop.stop()

