package frc.robot.util.csv;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public final class CSVWriter {

    private String headers;
    private List<List<Double>> dataLines = new ArrayList<>();
    private String filePath = new String();
    private String directory = new String();

    public CSVWriter(String root) {
        this.directory = root;
    }

    public void setFileName(String fileName) {
        this.filePath = directory + fileName + ".csv";
    }

    public void deleteCopy() {
        try {
            Files.deleteIfExists(Paths.get(this.filePath));
        } catch (IOException e) {
            System.err.println("CSV File deletion Failed!");
            return;
        }
    }

    public void setHeader(String inputHeaders) {
        this.headers = inputHeaders;
    }

    public void importData(List<List<Double>> data) {
        this.dataLines = data;
    }

    public void output() throws IOException {

        File csvOutputFile = new File(this.filePath);
        try (PrintWriter pw = new PrintWriter(csvOutputFile)) {
            pw.println(this.headers);
            this.dataLines.stream().map(this::convertToCSV).forEach(pw::println);
        } catch (Exception e) {
            return;
        }
        if (!csvOutputFile.exists())
            System.err.println("CSV Output Failed!");
    }

    private String convertToCSV(List<Double> data) {
        String input = data.toString().replace("[", "").replace("]", "").trim();
        return Stream.of(input).collect(Collectors.joining(",")).replace(" ", "");
    }
}