from mit_protocol_model import BimanualConsumerModel, ConsumerModel, Status, TupleCommand


def command(generation=1, session=7, lease=3, value=0.0):
    seven = (value,) * 7
    return TupleCommand(session, generation, lease, seven, seven, seven, seven, seven)


def test_ack_requires_matching_session_whole_valid_tuple_and_new_generation():
    consumer = ConsumerModel(session_id=7, max_lease_cycles=4)
    assert not consumer.accept(command(session=6))
    assert consumer.ack_generation == 0
    assert consumer.accept(command())
    assert consumer.ack_generation == 1
    assert not consumer.accept(command())


def test_lease_is_capped_and_stale_is_fail_closed():
    consumer = ConsumerModel(session_id=7, max_lease_cycles=3)
    assert not consumer.accept(command(lease=4))
    assert consumer.accept(command(lease=3))
    for _ in range(3):
        consumer.write_cycle_without_commit()
    assert consumer.status is Status.STALE


def test_external_switch_latch_blocks_new_active_command_and_deactivate_disables():
    consumer = ConsumerModel(session_id=7)
    assert consumer.accept(command())
    consumer.prepare_external_stop()
    assert consumer.status is Status.SAFE_TRANSITION
    assert not consumer.accept(command(generation=2))
    assert consumer.safe_ack_generation < consumer.safe_generation
    assert consumer.submit_hardware_safe()
    assert consumer.status is Status.SAFE
    assert consumer.safe_ack_generation == consumer.safe_generation
    consumer.deactivate()
    assert consumer.status is Status.DISABLED


def test_bimanual_commit_is_one_transaction_and_both_session_faults_together():
    consumer = BimanualConsumerModel(session_id=7)
    assert not consumer.accept_both(command(generation=1), command(generation=2))
    assert consumer.left.ack_generation == consumer.right.ack_generation == 0
    assert consumer.accept_both(command(generation=1), command(generation=1))
    assert consumer.left.ack_generation == consumer.right.ack_generation == 1
    consumer.fault_arm("left", both_arms_session=True)
    assert consumer.left.status is consumer.right.status is Status.FAULT


def test_independent_arm_fault_mandates_faulty_safe_and_defaults_peer_to_hold():
    consumer = BimanualConsumerModel(session_id=7)
    assert consumer.accept_both(command(), command())
    consumer.fault_arm("left", both_arms_session=False)
    assert consumer.left.status is Status.FAULT
    assert consumer.right.status is Status.SAFE_TRANSITION
    assert consumer.right.submit_hardware_safe()
    assert consumer.right.status is Status.SAFE


def test_generation_restart_requires_new_consumer_session_echo():
    old = ConsumerModel(session_id=7)
    assert old.accept(command(generation=9, session=7))
    restarted = ConsumerModel(session_id=8)
    assert not restarted.accept(command(generation=10, session=7))
    assert restarted.accept(command(generation=1, session=8))


def test_bimanual_preflight_never_partially_acks_latched_or_faulted_peer():
    for peer_condition in ("latched", "faulted"):
        consumer = BimanualConsumerModel(session_id=7)
        if peer_condition == "latched":
            consumer.right.prepare_external_stop()
        else:
            consumer.right.hardware_fault()
        assert not consumer.accept_both(command(), command())
        assert consumer.left.ack_generation == 0
        assert consumer.right.ack_generation == 0
