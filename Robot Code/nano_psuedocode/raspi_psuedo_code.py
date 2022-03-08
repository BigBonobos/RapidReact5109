# Preprossescor Directives
from networktables import NetworkTables


def main():
    nt_conn = NetworkTables.initialize(server='roborio-5109-frc.local')
    testTable = NetworkTables.getTable("tester")
    testTable.putNumber("shit", 5)

if (__name__ == "__main__"):
    main()