import rospy

from substrateinterface import SubstrateInterface, Keypair


def create_substrate_interface(url: str) -> SubstrateInterface:
    return SubstrateInterface(
        url=url,
        ss58_format=32,
        type_registry_preset="substrate-node-template",
        type_registry={
            "types": {
                "Record": "Vec<u8>",
                "<T as frame_system::Config>::AccountId": "AccountId",
                "RingBufferItem": {
                    "type": "struct",
                    "type_mapping": [
                        ["timestamp", "Compact<u64>"],
                        ["payload", "Vec<u8>"],
                    ],
                },
            }
        }
    )


class DataLogger:
    def __init__(self, mnemonic: str, substrate: SubstrateInterface):
        self._keypair = Keypair.create_from_mnemonic(mnemonic, ss58_format=32)
        self._substrate = substrate
        rospy.loginfo("DataLogger instantiated")

    def write(self, message: str) -> str:
        rospy.loginfo(f"Sending to datalog: {message}")
        call = self._substrate.compose_call(
            call_module="Datalog",
            call_function="record",
            call_params={"record": message},
        )
        extrinsic = self._substrate.create_signed_extrinsic(
            call=call,
            keypair=self._keypair
        )
        receipt = self._substrate.submit_extrinsic(extrinsic, wait_for_inclusion=True)
        rospy.loginfo(f"Datalog created with extrinsic hash: {receipt.extrinsic_hash}")
        return receipt.extrinsic_hash
