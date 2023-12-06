from typing import Any

import json
from abc import ABCMeta
from dataclasses import dataclass


class Gains:
    pass


class OSLJSONEncoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, ABCMeta):
            return o.__name__
        elif isinstance(o, Gains):
            return o.__dict__
        return json.JSONEncoder.default(self, o)


class OSLMsg:
    """Class for OSL socket communication.

    The class holds the different fields of the messages. It allows for reading and writing of the fields.

    Methods:

            encode (bytearray): Encodes the OSLMsg fields into a JSON dict in bytearray format. It append and prepends the message with headers and delimiter.
            decode: Decodes a bytearray containing an OSLMsg into the OSLMsg fields of the class.

    """

    uid: int
    type: str
    data: Any

    def __init__(self, uid: int, type: str, data: Any):
        self.uid = uid
        self.type = type
        self.data = data

    def encode(self) -> bytearray:
        """Encodes the OSLMsg fields into a JSON dict in bytearray format.

        Returns:
            bytearray: The bytearray containing the encoded message.

        """

        # Create a JSON dict from the OSLMsg fields
        json_dict = self.__dict__

        # Convert the JSON dict to a JSON string
        json_string = json.dumps(json_dict)

        # Convert the JSON string to a bytearray and return
        return bytearray(json_string, "utf-8")

    @classmethod
    def decode(cls, payload: bytearray) -> "OSLMsg":
        """Verify that the bytearray contains a valid OSLMsg and decode it into the OSLMsg fields of the instance.

        Args:
            payload (bytearray): The bytearray containing the encoded message.
        """
        # Convert the JSON payload to a JSON dict
        json_dict = json.loads(payload)
        msg = OSLMsg(None, None, None)

        # Check if the message contains the correct fields
        if not all(key in json_dict for key in msg.__dict__):
            raise Exception("Message does not contain all fields")

        # Update the OSLMsg fields
        msg.__dict__ = json_dict
        return msg

    def __str__(self):
        return str(self.__dict__)

    def __repr__(self):
        return str(self.__dict__)


class SocketIOFrame:
    """Class for OSL socket communication.

    The class holds the different fields of the messages. It allows for reading and writing of the fields.

    The message can be encoded by calling the encode method which will parse the fields in the __dict__ to a JSON
    string in bytearray form. The bytearray is appended and prepended with message headers and delimiter.

    Frame format:       <header> <len> <msg> <delimiter>
    Frame byte order:   <SOH> <len[15:8]><len[7:0]> <msg[0]><msg[1]>...<msg[len-1]> <CR><LF>

    Methods:

            encode (bytearray): Encodes the OSLMsg fields into a JSON dict in bytearray format. It append and prepends the message with headers and delimiter.
            decode list[OSLMsg]: Decodes a bytearray containing 1 or more OSLMsgs into a list of OSLMsgs.

    """

    HEADER = b"\x01"  # <SOH>
    DELIMITER = b"\r\n"
    LEN_FIELD_SIZE = 2
    ENDIANNESS = "big"
    FRAME_MIN_SIZE = len(HEADER) + LEN_FIELD_SIZE + len(DELIMITER)

    @classmethod
    def encode(cls, msg: OSLMsg) -> bytearray:
        """Encodes the OSLMsg fields into a JSON dict in bytearray format. It append and prepends the message with headers and delimiter.

        Returns:
            bytearray: The bytearray containing the encoded message.

        """
        # Convert the JSON string to a bytearray
        json_bytearray = bytearray(
            json.dumps(msg.__dict__, cls=OSLJSONEncoder), "utf-8"
        )

        payload_len = len(json_bytearray)

        # Append and prepend the bytearray with headers and delimiter
        json_bytearray = (
            cls.HEADER
            + payload_len.to_bytes(cls.LEN_FIELD_SIZE, cls.ENDIANNESS)
            + json_bytearray
            + cls.DELIMITER
        )

        return bytearray(json_bytearray)

    @classmethod
    def decode(cls, databuffer: bytearray) -> tuple[list[OSLMsg], bytearray]:
        """Parse a bytearray and return all valid OSLMsgs found in the bytearray.

        The buffer might contain more data than a single message. The function looks for the correct header and
        reads the <len> field to determine the length of the message. If a valid JSON string is found in
        the message payload it is decoded and added to the list of decoded messages.

        The databuffer is cleared of any successfully parsed or invalid data and should
        be checked for remaining data after the function returns.

        Args:
            json_payload (bytearray): A bytearray containing 0 or more message frames.

        """

        decoded_msgs = []

        while databuffer:
            # print(f"Decoding buffer: {databuffer}")
            ## Find n where databuffer[n] == <SOH>
            start_of_frame = databuffer.find(cls.HEADER)

            if start_of_frame == -1:
                # No header found, clear all data in buffer
                print("No header found, clearing buffer")
                databuffer.clear()
                continue

            # print(f"Found start of frame at index {start_of_frame}")

            # Read the msg length field <SOH><len[15:8]><len[7:0]><...> as a integer
            len_field = databuffer[
                start_of_frame + 1 : start_of_frame + cls.LEN_FIELD_SIZE + 1
            ]
            msg_length = int.from_bytes(len_field, cls.ENDIANNESS)

            # print(f"msg_length decoded: {msg_length}")

            start_of_message = start_of_frame + len(cls.HEADER) + cls.LEN_FIELD_SIZE
            end_of_message = start_of_message + msg_length
            end_of_frame = end_of_message + len(cls.DELIMITER)

            # print(f"Total frame size {end_of_frame - start_of_frame}")

            # Check if the buffer can contain a valid OSLMsg
            if len(databuffer) < end_of_message:
                print("Buffer to short to contain a valid OSLMsg")
                databuffer = databuffer[start_of_frame:]
                break

            # Check if the message contains the correct delimiter

            if databuffer[end_of_message:end_of_frame] != cls.DELIMITER:
                delimiter = databuffer[end_of_message:end_of_frame]
                databuffer = databuffer[start_of_frame + 1 :]
                print(
                    f"Invalid delimiter found: {delimiter} when expecting {cls.DELIMITER}"
                )
                continue

            # Get the message payload
            payload = databuffer[start_of_message:end_of_message]

            try:
                msg = OSLMsg.decode(payload)
            except Exception as e:
                print(f"Error decoding JSON payload {payload}")
                databuffer = databuffer[start_of_frame + 1 :]
                continue

            # Message decoded successfully, add it to the list
            decoded_msgs.append(msg)

            # Remove the sucessfully decoded frame from the buffer
            databuffer = databuffer[end_of_frame:]
            # print(f"Decoded message: {msg} \r\n")

        return (decoded_msgs, databuffer)

    def __str__(self) -> str:
        """Returns the OSLMsg fields as a string.

        Returns:
            str: The OSLMsg fields as a string.

        """

        return str(self.__dict__)


if __name__ == "__main__":

    ## Test the encode and decode functions

    # Create a message
    frame_bytearray = SocketIOFrame.encode(
        OSLMsg(1, "SET", {"FUN": 5.55555555555555555555555555555})
    )
    frame_bytearray += SocketIOFrame.encode(OSLMsg(2, "SET", {"FUN": 5}))
    frame_bytearray += bytearray(b"err") + SocketIOFrame.encode(
        OSLMsg(3, "SET", {"FUN": 5})
    )
    frame_bytearray.pop(-4)
    frame_bytearray += bytearray(b"err") + SocketIOFrame.encode(
        OSLMsg(3, "SET", {"FUN": 5})
    )
    frame_bytearray = frame_bytearray[0:-25]

    assert frame_bytearray[0] == 1

    # Decode the message
    messages, frame_bytearray = SocketIOFrame.decode(frame_bytearray)

    print(f"Returned buffer: {frame_bytearray}")

    # Print the decoded message
    print(messages)

    # Check that the decoded message is correct
