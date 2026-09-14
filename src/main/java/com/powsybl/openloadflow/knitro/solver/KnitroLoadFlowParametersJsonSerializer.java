/**
 * Copyright (c) 2025, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
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

import java.io.IOException;

/**
 * @author Salomé Lavine {@literal <salome.lavine at artelys.com>}
 */
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
     * Specifies serialization for our extension: ignore name and extendable.
     */
    private interface SerializationSpec {

        @JsonIgnore
        String getName();

        @JsonIgnore
        LoadFlowParameters getExtendable();
    }

    private static ObjectMapper createMapper() {
        return JsonUtil.createObjectMapper()
            .addMixIn(KnitroLoadFlowParameters.class, KnitroLoadFlowParametersJsonSerializer.SerializationSpec.class);
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
