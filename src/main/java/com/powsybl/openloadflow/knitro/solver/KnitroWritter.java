package com.powsybl.openloadflow.knitro.solver;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class KnitroWritter {
    private final String logFile;

    public KnitroWritter(String logFile) {
        this.logFile = logFile;
    }

    public void write(String message, boolean append) {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(logFile, append))) {
            writer.write(message);
            writer.newLine();
        } catch (IOException e) {
            System.err.println("Erreur lors de l'écriture des logs : " + e.getMessage());
        }
    }

    public String getLogFile() {
        return logFile;
    }
}
