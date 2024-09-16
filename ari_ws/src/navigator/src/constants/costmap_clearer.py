## The constants for the costmap_clearer node ##

from std_srvs.srv import Empty, EmptyRequest

SERVICE_NAME = "/move_base/clear_costmaps"
SERVICE_TYPE = Empty
SERVICE_REQUEST = EmptyRequest()
RATE = 1/10  # Hz
