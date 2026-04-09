# Exploration status constants
# EXPLORATION_STARTED: Exploration action accepted and planning started
# NAVIGATING_TO_FRONTIER: A new Nav2 goal has been sent to a frontier target
# EXPLORATION_COMPLETE: Exploration finished successfully
# RETURNING_TO_ORIGIN: Robot navigating back to initial pose
# RETURNED_TO_ORIGIN: Robot successfully reached initial pose
# EXPLORATION_CANCELED: Exploration was canceled by the action client
# EXPLORATION_ABORTED: Exploration failed and the action aborted

string EXPLORATION_STARTED=exploration_started
string NAVIGATING_TO_FRONTIER=navigating_to_frontier
string EXPLORATION_COMPLETE=exploration_complete
string RETURNING_TO_ORIGIN=returning_to_origin
string RETURNED_TO_ORIGIN=returned_to_origin
string EXPLORATION_CANCELED=exploration_canceled
string EXPLORATION_ABORTED=exploration_aborted

# Current exploration status
string status
