package com.powsybl.openloadflow.knitro.solver;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectReader;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.google.auto.service.AutoService;
import com.powsybl.commons.extensions.ExtensionJsonSerializer;
import com.powsybl.commons.json.JsonUtil;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowParameters;

import java.io.IOException;

@AutoService(ExtensionJsonSerializer.class)
public class KnitroLoadFlowParametersJsonSerializer implements ExtensionJsonSerializer<LoadFlowParameters, KnitroLoadFlowParameters> {

    @Override
    public void serialize(KnitroLoadFlowParameters extension, JsonGenerator jsonGenerator, SerializerProvider serializerProvider) throws IOException {
        createMapper().writeValue(jsonGenerator, extension);
    }

    @Override
    public KnitroLoadFlowParameters deserialize(JsonParser jsonParser, DeserializationContext deserializationContext) throws IOException {
        return createMapper().readValue(jsonParser, KnitroLoadFlowParameters.class);
    }

    /**
     * Specifies serialization for our extension: ignore name et extendable
     */
    private interface SerializationSpec {

        @JsonIgnore
        String getName();

        @JsonIgnore
        OpenLoadFlowParameters getExtendable();
    }

    private static ObjectMapper createMapper() {
        return JsonUtil.createObjectMapper()
            .addMixIn(OpenLoadFlowParameters.class, KnitroLoadFlowParametersJsonSerializer.SerializationSpec.class);
    }

    @Override
    public String getExtensionName() {
        return "knitro-load-flow-parameters";
    }

    @Override
    public String getCategoryName() {
        return "loadflow-parameters";
    }

    @Override
    public Class<? super KnitroLoadFlowParameters> getExtensionClass() {
        return KnitroLoadFlowParameters.class;
    }

    @Override
    public KnitroLoadFlowParameters deserializeAndUpdate(JsonParser jsonParser, DeserializationContext deserializationContext, KnitroLoadFlowParameters extension) throws IOException {
        ObjectMapper objectMapper = createMapper();
        ObjectReader objectReader = objectMapper.readerForUpdating(extension);
        return objectReader.readValue(jsonParser, KnitroLoadFlowParameters.class);
    }
}
